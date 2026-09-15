# Tài Liệu Giải Thích Bộ Điều Khiển PX4 Gốc & Custom Precision Landing Controller

Tài liệu này tổng hợp toàn bộ lý thuyết, kiến trúc và phân tích chi tiết về bộ điều khiển nối tầng của **PX4 Autopilot gốc** cũng như **Bộ điều khiển Custom Precision Landing (ROS 2 Package)**.

---

## 1. Tổng Quan Bộ Điều Khiển PX4 Gốc (PX4 Native Cascaded Control)

PX4 sử dụng kiến trúc **Điều khiển nối tầng (Cascaded Control Architecture)** phân cấp từ vòng ngoài (vị trí) đến vòng trong cùng (tốc độ góc).

```text
[Position Setpoint] 
       │
       ▼
┌────────────────────────┐
│  Position Controller   │  (P Controller - mc_pos_control)
└──────────┬─────────────┘
           │ Output = Velocity Setpoint (v_sp)
           ▼
┌────────────────────────┐
│  Velocity Controller   │  (PID Controller - mc_pos_control)
└──────────┬─────────────┘
           │ Output = Thrust Vector Setpoint & Attitude Setpoint (q_sp)
           ▼
┌────────────────────────┐
│  Attitude Controller   │  (P Controller - mc_att_control)
└──────────┬─────────────┘
           │ Output = Angular Rate Setpoint (p_sp, q_sp, r_sp) [rad/s]
           ▼
┌────────────────────────┐
│    Rate Controller     │  (Full PID Controller - mc_rate_control)
└──────────┬─────────────┘
           │ Output = Vehicle Torque Setpoint (τ_x, τ_y, τ_z) & Thrust Setpoint (T)
           ▼
┌────────────────────────┐
│ Control Allocator/Mixer│  (Chuyển đổi hình học khung drone)
└──────────┬─────────────┘
           │ Output = Actuator Motor Outputs (Tín hiệu PWM / DShot)
           ▼
┌────────────────────────┐
│     PHYSICAL DRONE     │  (System / Plant)
└────────────────────────┘
```

### Chi tiết tín hiệu Đầu vào / Đầu ra (Input / Output) của từng tầng:

1. **System Input của Hệ Vật Lý (Drone Plant)**:
   * **Mặt toán học / động học**: Đầu ra của bộ `Rate Controller` (PID vòng cuối) chính là **Mô-men chuẩn hóa (`vehicle_torque_setpoint`: $\tau_x, \tau_y, \tau_z \in [-1, 1]$)** xung quanh 3 trục thân (Roll, Pitch, Yaw) kết hợp với **Lực đẩy tổng chuẩn hóa (`vehicle_thrust_setpoint`: $T \in [0, 1]$)**.
   * **Mặt cơ cấu chấp hành (ESC / Motors)**: Sau bộ **Control Allocator (Mixer)**, các giá trị moment và lực đẩy được chuyển đổi thành **Tín hiệu PWM ($1000 - 2000\,\mu s$) hoặc DShot** cấp trực tiếp cho các ESC động cơ.

---

## 2. Phân Tích: Tại Sao Position Control PX4 Chỉ Dùng P Mà Không Dùng PID? Overshoot Thì Sao?

### A. Lý do Position Control chỉ sử dụng khâu P (Proportional)

#### 1️⃣ Bản chất Động học ($pos = \int vel \, dt \implies \dot{x} = v$)
* Vị trí là tích phân tự nhiên của vận tốc. Khi vòng vị trí dùng P-Gain:
  $$v_{sp} = K_p \cdot (x_{sp} - x)$$
  Giả sử vòng vận tốc bên trong bám setpoint tốt ($v \approx v_{sp}$), phương trình vi phân mô tả vòng vị trí là:
  $$\dot{x} = K_p \cdot (x_{sp} - x) \implies \dot{e} + K_p e = 0$$
* Đây là một **hệ vi phân tuyến tính bậc 1**. Về mặt lý thuyết, hệ bậc 1 với phản hồi P thuần túy **ổn định tuyệt đối và không bao giờ tự phát sinh dao động hay vọt lố (overshoot)**.

#### 2️⃣ Tránh xung đột Tích phân (Integrator Windup & Phase Lag)
* Khâu tích phân (I) dùng để triệt tiêu sai số do nhiễu lực (như gió). 
* Trong PX4, **khâu I đã nằm ở Vòng Vận tốc (`_gain_vel_i`)**. 
* Nếu thêm khâu I ở vòng vị trí ngoài, hai khâu tích phân nối tầng sẽ "tranh giành" nhau tích lũy sai số, làm tăng **độ trễ pha (phase lag)** và gây ra dao động lắc lư hệ thống.

#### 3️⃣ Khâu D của Vị trí là thừa
* Đạo hàm vị trí $\frac{d}{dt}(e_{pos}) = -v$. 
* Tác dụng cản theo vận tốc này **chính là khâu P của Vòng Vận tốc bên trong**. Vì vậy, việc điều khiển vận tốc ở vòng trong đã tự động đóng vai trò là khâu D (damping) cho vòng vị trí ngoài.

---

### B. Hiện tượng Vọt Lố (Overshoot) được ngăn chặn như thế nào?

Mặc dù vòng vị trí chỉ dùng P, PX4 triệt tiêu hoàn toàn nguy cơ Overshoot nhờ các cơ chế:

1. **Damping từ Vòng Vận tốc (Velocity PID)**: Khâu $P_{vel}$ và $D_{vel}$ của vòng vận tốc bên trong tạo ra lực hãm phanh ngược lại rất mạnh khi drone tiến gần về vị trí đích.
2. **Giới hạn Vận tốc (`ControlMath::constrainXY`)**: Tín hiệu $v_{sp} = K_p \cdot e$ luôn bị khống chế qua hàm `constrain`, không cho phép vận tốc vượt quá giới hạn an toàn.
3. **Bộ tạo quỹ đạo S-Curve (Jerk-Limited Trajectory Generator)**: Quy hoạch đường đi mượt mà với giới hạn Gia tốc (Acceleration) và Độ giật (Jerk), tự động phát lệnh giảm tốc trước khi đến điểm dừng.
4. **Feed-Forward Vận tốc & Gia tốc**: Giúp drone bám đường bay động chuẩn xác mà không cần đẩy gain $K_p$ quá cao.

---

## 3. Bộ Điều Khiển Custom Precision Landing (ROS 2 Package)

Gói mã nguồn [`precision_landing`](file:///home/teedee/PX4/examples/SITL_PrecisionLanding/ros2_ws/src/precision_landing) chạy trên ROS 2 đóng vai trò là **Bộ điều khiển chỉ dẫn thị giác cấp cao (High-Level Visual Servo Guidance Controller)**.

### A. Các thuật toán được sử dụng trong Package
1. **Máy trạng thái hữu hạn (FSM - 9 trạng thái)**: Quản lý chu trình hạ cánh an toàn (`IDLE`, `START`, `HORIZONTAL_APPROACH`, `DESCEND_ABOVE_TARGET`, `FINAL_APPROACH`, `SEARCH`, `TARGET_LOST`, `FALLBACK`, `DONE`).
2. **Proportional Visual Servoing (Bộ điều khiển P thị giác)**: Tính toán sai số vị trí tương đối giữa drone và thảm ArUco:
   $$\Delta x = K_{servo} \cdot e_x, \quad \Delta y = K_{servo} \cdot e_y$$
3. **Gain Scheduling theo độ cao ($K_{servo}(z)$)**: Gain $K_{servo}$ thay đổi linh hoạt theo độ cao hiện tại của drone (cao thì gain nhỏ để bay mượt, thấp thì gain lớn để căn tâm tỉ mỉ).
4. **Kinematic Slew-Rate Limiter**: Giới hạn tốc độ và gia tốc biến thiên của Setpoint (`apply_slew_rate()`) tránh làm Setpoint bị giật đột ngột.
5. **Circular Mean Yaw Alignment**: Xoay góc Yaw của drone bám theo hướng của thảm ArUco bằng thuật toán tính trung bình góc phẳng.

---

### B. Không có tham số I, D ở ROS 2 thì có chống gió được trong chế độ OFFBOARD không?

**CÓ, DRONE VẪN CHỐNG GIÓ BÌNH THƯỜNG VÀ RẤT TỐT!**

#### Lý do:
* Node ROS 2 chỉ làm nhiệm vụ tính toán và gửi **Tọa độ vị trí đích (Position Setpoint $x_{sp}, y_{sp}, z_{sp}$)** qua Offboard Mode sang PX4.
* Khi PX4 nhận được Position Setpoint từ ROS 2:
  1. PX4 đưa Setpoint này vào vòng **Velocity PID (`mc_pos_control`)** nội tại bên dưới của PX4.
  2. Nếu có gió thổi làm drone bị đẩy trôi, khâu **Integrator ($I_{vel}$)** của PX4 sẽ tự động tích lũy sai số.
  3. Khâu $I_{vel}$ của PX4 tự động ra lệnh nghiêng góc thân drone (Thrust Vector Tilt) để **tạo lực đẩy nghiêng chống lại đúng lực gió**.

#### Phân công vai trò:
* **Node ROS 2 (precision_landing)**: Đóng vai trò là **"Mắt"** (Nhìn camera và bảo drone: *"Hãy đi tới tọa độ $X_{sp}, Y_{sp}$!"*).
* **PX4 Autopilot**: Đóng vai trò là **"Bắp thịt & Hệ thần kinh"** (Thực thi lệnh, tự động nghiêng người kháng lại lực gió bằng bộ PID sẵn có).

---

### C. Tác dụng thực chất của bộ điều khiển P trong Package ROS 2

Bộ điều khiển P trong gói ROS 2 có tác dụng: **ĐIỀU KHIỂN CĂN TÂM THỊ GIÁC (Visual Servo Centering Controller)**.

* **Sự khác biệt với PX4 mặc định**: PX4 chỉ biết hạ cánh theo GPS (có sai số từ 1 - 3 mét). Node ROS 2 nhìn thấy thảm ArUco qua camera và dùng bộ P liên tục ép sai số lệch tâm $e = (x_{ArUco} - x_{Drone}) \to 0$.
* Giúp hạ cánh chính xác vào tâm tấm thảm ArUco với **sai số chỉ vài centimet**.

---

### D. Tại sao không truyền thẳng giá trị $(X, Y, Z)$ ($K_p = 1.0$) sang PX4 luôn?

Việc gán thẳng $X_{sp} = X_{ArUco}$ tương đương với việc đặt $K_p = 1.0$. Trong thực tế **KHÔNG BAO GIỜ được làm như vậy** vì 4 lý do:

1. **Tránh làm nghiêng Drone gắt gây MẤT DẤU CAMERA (Out of FOV)**: 
   * Camera được gắn dưới bụng drone. 
   * Ở độ cao 10m nếu lệch 2m, nếu gán ngay $X_{sp} = X_{ArUco}$ ($K_p=1$), PX4 sẽ nghiêng drone rất gắt ($20^\circ - 30^\circ$) để lao tới. 
   * Thân drone nghiêng gắt sẽ hất Camera nghiêng theo $\rightarrow$ Thảm ArUco bị văng ra khỏi khung hình camera $\rightarrow$ **Mất dấu mục tiêu, hạ cánh thất bại!**
   * Với $K_p < 1.0$ (ví dụ $K_p = 0.3$), drone chỉ di chuyển từng bước nhỏ mượt mà, giữ drone luôn bay phẳng và camera luôn hướng về thảm.
2. **Triệt tiêu NHIỄU RUNG của Camera (Visual Jitter / Noise)**: Tọa độ ArUco từ camera bị giật rung vài mm/cm giữa các khung hình. $K_p < 1$ đóng vai trò làm mượt (Smoothing Filter), tránh làm động cơ drone bị rung giật 30-60Hz.
3. **Tự động điều chỉnh độ nhạy theo độ cao (Gain Scheduling $K_p(z)$)**: Sai số 1m ở độ cao 15m khác hoàn toàn sai số 10cm ở độ cao 0.5m. $K_p(z)$ giúp phản ứng hợp lý ở mọi độ cao.
4. **Tạo đường cong tiếp cận mượt mà (Exponential Decay)**: Giúp drone giảm tốc từ từ và đáp nhẹ nhàng đúng tâm thảm.
