# PX4 Gimbal Precision Landing

Project này chứa pipeline hạ cánh chính xác cho drone `x500_gimbal` trong mô phỏng Gazebo SITL sử dụng **Fractal ArUco landing** (bộ tracker C++ `aruco_fractal_tracker` với cấu trúc marker lồng nhau nested fractal marker tùy chỉnh có kích thước ngoài cùng 50 cm), sử dụng **MAVROS** làm giao thức kết nối điều khiển chính.

---

## Cấu Trúc Thư Mục & Đồng Bộ Mô Phỏng

1. **Clone Dự Án**: Clone repository này vào thư mục `examples` của cây thư mục PX4 checkout:

   ```bash
   cd ~/PX4/examples
   git clone git@github.com:TeedeeTD/SITL_PrecisionLanding.git
   ```

   Cấu trúc thư mục mong đợi:
   ```text
   ~/PX4
   └── examples
       └── SITL_PrecisionLanding
   ```

2. **Đồng Bộ Hóa Mô Phỏng (Sync Worlds, Models & Textures)**:
   PX4 Gazebo sẽ load các world và model từ thư mục nội bộ của PX4. Đồng bộ hóa toàn bộ tài nguyên mô phỏng (bao gồm các file world `.sdf`, mô hình `x500_gimbal`, mô hình marker Fractal `fractal_aruco_marker`, và mô hình box `dib_box_landing_pad`) bằng lệnh `rsync`:

   ```bash
   cd ~/PX4
   rsync -a \
     examples/SITL_PrecisionLanding/px4/Tools/simulation/gz/ \
     Tools/simulation/gz/
   ```

3. **Kiểm tra các tệp tin chính**:

   ```bash
   # Kiểm tra worlds
   ls ~/PX4/Tools/simulation/gz/worlds/fractal_aruco_landing.sdf

   # Kiểm tra mô hình và kết cấu ảnh của Fractal
   ls ~/PX4/Tools/simulation/gz/models/fractal_aruco_marker/model.sdf
   ls ~/PX4/Tools/simulation/gz/models/fractal_aruco_marker/marker.png
   ls ~/PX4/Tools/simulation/gz/models/fractal_aruco_marker/custom_fractal.yml

   # Kiểm tra mô hình box landing pad
   ls ~/PX4/Tools/simulation/gz/models/dib_box_landing_pad/model.sdf
   ```

---

## Yêu Cầu

Cần có:

- PX4 Gazebo simulation chạy được.
- PX4 `gz_x500_gimbal` chạy được.
- ROS 2 Humble.
- MAVROS cho các pipeline điều khiển hạ cánh chính xác.
- `ros_gz_image`, `cv_bridge`, `rqt_image_view`.
- ArUco C++ library có `libaruco.so.3.1`.

### Cách kiểm tra môi trường & cài đặt tự động (Khuyên dùng)

Để đảm bảo mọi thư viện (bao gồm `libaruco`, các package ROS 2, Python dependencies và Gazebo bridge thích hợp) đều được cài đặt chính xác, bạn chỉ cần chạy script kiểm tra tự động đi kèm trong thư mục dự án:

```bash
cd ~/PX4/examples/SITL_PrecisionLanding
chmod +x verify_build_env.sh
./verify_build_env.sh
```

Script này sẽ tự động:
1. Phát hiện phiên bản Gazebo của bạn để cài đặt đúng gói bridge.
2. Kiểm tra và tự động cài các package ROS 2 còn thiếu.
3. Giải nén và tự động biên dịch thư viện C++ `libaruco 3.1.12` từ file đính kèm nếu hệ thống chưa có.
4. Cài đặt các package Python thông qua `requirements.txt`.

## Hoặc nếu bạn muốn tự cài đặt thủ công, hãy làm theo các bước dưới đây:

### Cài package ROS 2 thường dùng

Tùy thuộc vào phiên bản Gazebo được cài đặt trên máy của bạn (ví dụ: Gazebo Garden hoặc Gazebo Harmonic), chọn cài đặt các gói bridge tương thích:

**Với Gazebo Garden (Mặc định của PX4 v1.14+):**
```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz-image \
  ros-humble-ros-gz-bridge
```

**Với Gazebo Harmonic (Dành cho các hệ thống máy mới):**
```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gzharmonic-image \
  ros-humble-ros-gzharmonic-bridge
```

**Cài đặt các gói ROS 2 bổ sung và MAVROS:**
```bash
sudo apt install -y \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rqt-image-view \
  python3-colcon-common-extensions \
  python3-opencv
```

**Cài đặt thư viện Python thông qua `requirements.txt`:**
```bash
pip3 install -r requirements.txt
```

> [!NOTE]
> **Khắc phục lỗi không tương thích OpenCV 4 (OpenCV 4.7+):**
> Trong file `aruco_standard_tracker_node.cpp`, hàm `cv::aruco::drawAxis` (đã bị xóa trên OpenCV mới) được cập nhật bằng `cv::drawFrameAxes` để đảm bảo code tự động biên dịch thành công trên mọi môi trường máy mới.


---

## Build

`px4_msgs` cần nằm trong workspace hoặc được source từ workspace khác:

```bash
cd ~/PX4/examples/SITL_PrecisionLanding/ros2_ws
source /opt/ros/humble/setup.bash

# Nếu chưa có px4_msgs:
git clone https://github.com/PX4/px4_msgs.git src/px4_msgs

colcon build --symlink-install
source install/setup.bash
```

Nếu tracker thiếu `libaruco.so.3.1`, kiểm tra:

```bash
ldd install/aruco_fractal_tracker/lib/aruco_fractal_tracker/aruco_fractal_tracker | grep aruco
```

Nếu chưa resolve tới `/home/teedee/.local/lib/libaruco.so.3.1`, rebuild tracker:

```bash
cd ~/PX4/examples/SITL_PrecisionLanding/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select aruco_fractal_tracker --cmake-clean-cache
source install/setup.bash
```

---

## Dọn Tiến Trình Cũ

```bash
pkill -9 -f "gz sim|px4|mavros|tracker|lander|rqt_image_view|ros_gz"
```

---

## 1. Fractal ArUco Landing (MAVROS-based)

Pipeline định vị hạ cánh chính xác sử dụng MAVROS. Tracker C++ xuất contract `/landing/target_camera` (`dib_msgs/LandingTarget6D`) trong camera optical frame với state `LOST/SEARCHING/TRACKING`; topic pose `/aruco_fractal_tracker/poses` được giữ cho debug. Lander lọc target, bù camera offset, xoay theo yaw thân drone và điều khiển trong local ENU.

Cấu hình SITL hiện tại:

```text
box pose:     x=4.0, y=-3.5, z=0.0, yaw=0.0
marker:       0.50 m x 0.50 m, mounted on dib_box_landing_pad
camera:       1280 x 720, 30 Hz
horizontal FOV: 1.4137 rad (81°)
control loop: 30 Hz target, requirement >= 20 Hz
```
Kích thước vật lý thực tế của từng tầng:
Tầng ngoài cùng (Outer - Level 1): 50 cm (0.50 m)
Tầng giữa (Middle - Level 2): 12.5 cm (0.125 m)
Tầng trong cùng (Inner - Level 3): 3.125 cm (0.03125 m)

`dib_box_landing_pad/model.sdf`, `marker_size` trong launch và marker vật lý phải luôn dùng cùng kích thước. Detector sử dụng `custom_fractal.yml`; file này được sync sang PX4 cùng model.

* **Cập nhật file:**
  Ghi đè trực tiếp giá trị `<horizontal_fov>1.4137</horizontal_fov>` vào file
  ```bash
  /home/teedee/PX4/Tools/simulation/gz/models/gimbal/model.sdf trên đĩa.
  ```

  > [!WARNING]
  > **Lưu ý quan trọng khi chạy `make distclean`:**
  > Khi thực hiện `make distclean` hoặc dọn dẹp môi trường build PX4, file `model.sdf` của gimbal sẽ bị reset về giá trị mặc định. Bạn **bắt buộc phải kiểm tra và cập nhật lại** giá trị `<horizontal_fov>1.4137</horizontal_fov>` trong file `/home/teedee/PX4/Tools/simulation/gz/models/gimbal/model.sdf` trước khi khởi động lại mô phỏng SITL/HITL, nếu không node `aruco_fractal_tracker` sẽ sử dụng sai thông số FOV camera dẫn tới việc không thể ước lượng chính xác vị trí marker.


* **Terminal 1: Khởi động PX4 SITL**

  ```bash
  cd ~/PX4
  PX4_GZ_WORLD=fractal_aruco_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
  ```

* **Terminal 2: Chạy MAVROS một lần và giữ nguyên**

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14580
```

Kiểm tra MAVROS đã nối PX4:

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/SITL_PrecisionLanding/ros2_ws/install/setup.bash
ros2 topic echo --once /mavros/state
```

Kỳ vọng:

```text
connected: true
```

* **Terminal 3: Khởi động bridge camera, tracker và lander**

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/SITL_PrecisionLanding/ros2_ws/install/setup.bash
ros2 launch px4_offboard fractal_aruco_landing.launch.py
```

* **Terminal 4: Xem luồng camera có HUD overlay trực quan**

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```
*Chọn topic `/landing/annotated_image` từ thanh công cụ để theo dõi trực quan trạng thái FSM, tọa độ bám bắt, và các thông tin chẩn đoán trực tiếp.*



`fractal_aruco_landing.launch.py` không khởi động MAVROS. Khi cần sửa/restart tracker hoặc lander, chỉ restart Terminal 4. Không restart Terminal 2, như vậy PX4 vẫn nhận heartbeat mission computer từ MAVROS liên tục.

Nếu restart MAVROS trong lúc đang bay hoặc đang giữ OFFBOARD, QGroundControl/PX4 có thể báo:

```text
Critical: Connection to mission computer lost
```

Nếu PX4 checkout không nằm tại `~/PX4`, truyền đường dẫn cấu hình marker:

```bash
ros2 launch px4_offboard fractal_aruco_landing.launch.py \
  marker_configuration:=/absolute/path/to/custom_fractal.yml
```

Controller dùng ENU cho logic hạ cánh:

```text
search_x = East
search_y = North
pos_enu / target_enu / raw_enu / sp_enu đều là ENU
```

### 1.1 Box Hybrid Landing (SITL prototype)

Pipeline thử nghiệm cho flow `box_manager + precision landing` dùng cùng PX4 SITL, MAVROS và fractal tracker, nhưng thay lander cũ bằng FSM hybrid:

```text
IDLE -> DRONE_MISSION -> PRELANDING_CHECK -> WAIT_BOX_READY
     -> SEARCH -> HORIZONTAL_APPROACH -> DESCEND_OVER_TARGET
     -> LAND -> FLIGHT_IN_PROGRESS -> DONE
```

Trong prototype này, box thật được thay bằng `sim_box_manager`, publish `/sim_box/state`:

```text
IDLE -> PREPARING_FOR_LANDING -> WAITING_FOR_LANDING
```

Gazebo world đã có box tĩnh `dib_box_landing_pad` tại:

```text
x=4.0, y=-3.5, z=0.0, yaw=0.0
```

Trong Phase 2, vị trí này là fixture mô phỏng cho box/mission/marker. UAV nên bay tới vùng này bằng mission hoặc waypoint của box; hybrid lander chỉ bắt đầu visual refinement sau khi mission/prelanding đã hoàn tất, không dùng `search_x/search_y` để bay tới box.

Chạy PX4 SITL và MAVROS giống mục Fractal ArUco Landing ở trên. Terminal 3 đổi sang launch hybrid:

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/gimbal_simulation/ros2_ws/install/setup.bash
ros2 launch px4_offboard box_hybrid_landing.launch.py
```

Hybrid lander không tự khởi động mission và không tự bay OFFBOARD tới box trong state `DRONE_MISSION`. Hãy setup/khởi chạy mission bằng QGroundControl hoặc luồng mission thật, rồi gửi trigger để node bắt đầu monitor mission/box:

```bash
ros2 topic pub --once /box_hybrid_landing/trigger std_msgs/msg/String "data: 'land'"
```

Trong SITL, node dùng waypoint progress hoặc khoảng cách local tới box fixture `(4.0, -3.5)` để nhận biết đã tới vùng hạ cánh. `manual_drive_alt` mặc định là `10.0m`, đóng vai trò độ cao approach/visual acquire ban đầu. Chỉ sau đó nó mới chuẩn bị gimbal, gửi `REQUEST_LANDING` tới box và chuyển sang visual guidance.

Visual guidance mặc định dùng OFFBOARD setpoint sau khi đã tới box:

```bash
ros2 launch px4_offboard box_hybrid_landing.launch.py enable_offboard_visual_servo:=true
```

Kiểm tra FSM:

```bash
ros2 topic echo /box_hybrid_landing/state
ros2 topic echo /box_hybrid_landing/box_state
ros2 topic echo /box_hybrid_landing/comms
```

Yaw alignment hiện có guard:

```bash
ros2 launch px4_offboard box_hybrid_landing.launch.py enable_yaw_setpoint:=true yaw_gate_deg:=5.0
```

Chỉ bật sau khi đã xác nhận quyền điều khiển mode/setpoint không xung đột với PX4/MAVROS mission flow. Khi bật, yaw được align tại `final_alt` trong lúc giữ XY/altitude, rồi mới trigger `AUTO.LAND`. Có thể siết `yaw_gate_deg:=3.0` khi muốn test chính xác hơn.

---

## Giám Sát và Kiểm Tra

Quy trình nghiệm thu đầy đủ cho Fractal ArUco nằm ở:

```bash
~/PX4/examples/gimbal_simulation/docs/FLIGHT_TEST.md
```

FSM hiện tại của pipeline Fractal ArUco độc lập nằm ở:

```bash
~/PX4/examples/gimbal_simulation/docs/fractal_aruco_fsm.png
```

Proposal FSM cho hướng tích hợp mission-driven với `box_manager` nằm ở:

```bash
~/PX4/examples/gimbal_simulation/docs/main_fsm.mmd
~/PX4/examples/gimbal_simulation/docs/precision_landing_fsm.mmd
```

Kế hoạch mô phỏng SITL cho box-driven hybrid landing nằm ở:

```bash
~/PX4/examples/gimbal_simulation/docs/BOX_HYBRID_SITL_PLAN.md
```

* **Xem luồng camera có telemetry HUD**:
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 run rqt_image_view rqt_image_view
  ```
  Chọn topic `/landing/annotated_image` để xem hình ảnh bám bắt mục tiêu trực quan cùng thông tin telemetry (FPS, FSM State, coordinates, TVEC).

* **Kiểm tra trạng thái FSM của Lander**:
  ```bash
  ros2 topic echo /lander/state
  ```

* **Kiểm tra tần số Setpoint gửi đến PX4**:
  ```bash
  ros2 topic hz /mavros/setpoint_position/local
  ```
  *(Timer điều khiển chạy 30Hz; kết quả đo nghiệm thu cần đạt >=20Hz khi đang ở chế độ Offboard.)*

---

## Kiểm Tra Nhanh

Camera bridge:

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /gimbal_camera
```

Tracker target và pose debug:

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/SITL_PrecisionLanding/ros2_ws/install/setup.bash
ros2 topic hz /landing/target_camera
ros2 topic echo --once /landing/target_camera
ros2 topic hz /aruco_fractal_tracker/poses
ros2 topic echo --once /aruco_fractal_tracker/poses
```

MAVROS topics:

```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/SITL_PrecisionLanding/ros2_ws/install/setup.bash
ros2 topic hz /mavros/setpoint_position/local
ros2 topic echo --once /mavros/state
ros2 topic echo --once /mavros/extended_state
ros2 topic echo --once /mavros/local_position/pose
```

---

## Dấu Hiệu Thành Công

Landing thành công khi log có dạng:

```text
Marker detected
State: SEARCH -> HORIZONTAL_APPROACH
State: HORIZONTAL_APPROACH -> DESCEND_OVER_TARGET
Final altitude reached
PX4 land detector reports landed
LANDING COMPLETE
```

Không dùng force-disarm làm tiêu chuẩn thành công khi chạy thật.

---

## Ghi Chú

- Thống nhất kích thước tất cả các marker chính về `0.50m` (`marker_size:=0.50`).
- Nếu sau này đổi physical marker size trong `model.sdf`, phải đổi `marker_size` tương ứng trong file launch.
- `command 520 unsupported` là capability request MAVLink cũ từ client và không phải lệnh điều khiển landing.
- Trước một lần chạy sạch từ đầu, dừng các tiến trình PX4/MAVROS cũ để tránh giữ UDP endpoint hoặc quyền điều khiển gimbal từ phiên trước.
- Khi đang debug giữa chuyến bay hoặc đang giữ OFFBOARD, không restart MAVROS. Hãy giữ Terminal 2 chạy MAVROS riêng và chỉ restart Terminal 3 với `ros2 launch px4_offboard fractal_aruco_landing.launch.py`.

---

## 2. Hướng Dẫn Chạy Thực Tế & Mô Phỏng Với PX4 Precland

### 2.1. Chạy detect bằng cam thật:
```bash
source ~/PX4/examples/SITL_PrecisionLanding/ros2_ws/install/setup.bash
ros2 launch siyi_camera_bridge real_fractal_detect.launch.py \
  enable_mavros:=false \
  rtsp_url:=rtsp://192.168.168.16:8554/main.264 \
  marker_configuration:=/home/teedee/PX4/examples/SITL_PrecisionLanding/px4/Tools/simulation/gz/models/fractal_aruco_marker/custom_fractal.yml \
  marker_size:=0.162 \
  flip_180:=false
```

### 2.2. Chạy Precland điều khiển bằng PX4 (Sử dụng [qgc_sim_precland.launch.py](file:///home/teedee/PX4/examples/SITL_PrecisionLanding/ros2_ws/src/px4_offboard/launch/qgc_sim_precland.launch.py) và [landing_target_bridge.py](file:///home/teedee/PX4/examples/SITL_PrecisionLanding/ros2_ws/src/px4_offboard/px4_offboard/landing_target_bridge.py)):

#### Dọn Tiến Trình Cũ:
```bash
pkill -9 -f "gz sim|px4|mavros|tracker|lander|rqt_image_view|ros_gz"
```

#### Terminal 1: Khởi động PX4 SITL:
```bash
cd ~/PX4
PX4_GZ_WORLD=fractal_aruco_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
```

#### Terminal 2: Chạy MAVROS một lần và giữ nguyên:
```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14580
```
Kiểm tra MAVROS đã nối PX4:
```bash
source /opt/ros/humble/setup.bash
source ~/SITL_PrecisionLanding/ros2_ws/install/setup.bash
ros2 topic echo --once /mavros/state
```
Kỳ vọng:
```text
connected: true
```

#### Terminal 3: Khởi động bridge camera, tracker và lander:
```bash
source /opt/ros/humble/setup.bash
source ~/SITL_PrecisionLanding/ros2_ws/install/setup.bash
ros2 launch px4_offboard qgc_sim_precland.launch.py
```

#### Terminal 4: Xem luồng camera có HUD overlay trực quan:
```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```
*Chọn topic `/landing/annotated_image` từ thanh công cụ để theo dõi trực quan trạng thái FSM, tọa độ bám bắt, và các thông tin chẩn đoán trực tiếp.*

`qgc_sim_precland.launch.py` không khởi động MAVROS. Khi cần sửa/restart tracker hoặc lander, chỉ restart Terminal 3. Không restart Terminal 2, như vậy PX4 vẫn nhận heartbeat mission computer từ MAVROS liên tục.

### 2.3. Hướng Dẫn Chạy Precland Điều Khiển Bằng PX4 Chế Độ Offboard:
Sử dụng [qgc_offboard_precland.launch.py](file:///home/teedee/PX4/examples/SITL_PrecisionLanding/ros2_ws/src/px4_offboard/launch/qgc_offboard_precland.launch.py) và [offboard_precland_controller.py](file:///home/teedee/PX4/examples/SITL_PrecisionLanding/ros2_ws/src/px4_offboard/px4_offboard/offboard_precland_controller.py):

#### Dọn Tiến Trình Cũ:
```bash
pkill -9 -f "gz sim|px4|mavros|tracker|lander|rqt_image_view|ros_gz"
```

#### Terminal 1: Khởi động PX4 SITL:
```bash
cd ~/PX4
PX4_GZ_WORLD=fractal_aruco_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
```

#### Terminal 2: Chạy MAVROS một lần và giữ nguyên:
```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14580
```
Kiểm tra MAVROS đã nối PX4:
```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/SITL_PrecisionLanding/ros2_ws/install/setup.bash
ros2 topic echo --once /mavros/state
```
Kỳ vọng:
```text
connected: true
```

#### Terminal 3: Khởi động bridge camera, tracker và lander:
```bash
source /opt/ros/humble/setup.bash
source ~/PX4/examples/SITL_PrecisionLanding/ros2_ws/install/setup.bash
ros2 launch px4_offboard qgc_offboard_precland.launch.py
```

#### Terminal 4: Xem luồng camera có HUD overlay trực quan:
```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```
*Chọn topic `/landing/annotated_image` từ thanh công cụ để theo dõi trực quan trạng thái FSM, tọa độ bám bắt, và các thông tin chẩn đoán trực tiếp.*

`qgc_offboard_precland.launch.py` không khởi động MAVROS. Khi cần sửa/restart tracker hoặc lander, chỉ restart Terminal 3. Không restart Terminal 2, như vậy PX4 vẫn nhận heartbeat mission computer từ MAVROS liên tục.

### 2.4. Hướng Dẫn Chạy SITL Precland sử dụng pkg precision_landing (Phiên bản C++):
> **Lưu ý**: `dib_msgs` chỉ thêm `LandingTarget6D` so với bản dev.

#### Dọn Tiến Trình Cũ:
```bash
pkill -9 -f "gz sim|px4|mavros|tracker|lander|rqt_image_view|ros_gz"
```

#### Terminal 1: Khởi động PX4 SITL:
```bash
cd ~/PX4
PX4_GZ_WORLD=fractal_aruco_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
```

#### Terminal 2: Chạy MAVROS một lần và giữ nguyên:
```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14580
```
Kiểm tra MAVROS đã nối PX4:
```bash
source /opt/ros/humble/setup.bash
source ~/precision_landing_ws/install/setup.bash
ros2 topic echo --once /mavros/state
```
Kỳ vọng:
```text
connected: true
```

#### Terminal 3: Khởi động bridge camera, tracker và lander:
```bash
source /opt/ros/humble/setup.bash
source ~/precision_landing_ws/install/setup.bash
ros2 launch precision_landing sitl_precland.launch.py
```

#### Terminal 4: Xem luồng camera có HUD overlay trực quan:
```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```
Chọn topic `/landing/annotated_image` từ thanh công cụ để theo dõi trực quan trạng thái FSM, tọa độ bám bắt, và các thông tin chẩn đoán trực tiếp.

`sitl_precland.launch.py` không khởi động MAVROS. Khi cần sửa/restart tracker hoặc lander, chỉ restart Terminal 3. Không restart Terminal 2, như vậy PX4 vẫn nhận heartbeat mission computer từ MAVROS liên tục.

#### Gửi bài bay tự động qua service:

*   **Terminal 5**:
    ```bash
    source ~/test_req/install/setup.bash 
    ros2 launch ros2_telemetry plan_upload.launch.py drone_id:=d1
    ```

*   **Terminal 6**:
    ```bash
    source ~/test_req/install/setup.bash 
    ros2 service call /d1/mission_upload dib_msgs/srv/MissionUpload "{mission: [
      {command: 22, param1: 0.0, param2: 0.0,  latitude: 47.397929, longitude: 8.546217, altitude: 15.0},
      {command: 16, param1: 5.0, param2: 0.0, latitude: 47.39797, longitude: 8.546322, altitude: 15.0}, 
      {command: 16, param1: 5.0, param2: 0.0, latitude: 47.3979298, longitude: 8.546217, altitude: 15.0},
      {command: 23, param1: 0.0, param2: 0.0, latitude: 47.3979298, longitude: 8.546217, altitude: 0.0} 
    ]}"
    ```

---

## 2.5. Hướng Dẫn Chạy Mô phỏng HITL / Đa máy tính (PC chạy Gazebo + Jetson chạy Thuật toán)

Khi chạy mô phỏng cấu hình đa máy tính (PC chạy Gazebo SITL/HITL, Jetson đóng vai trò Companion Computer chạy toàn bộ pipeline nhận diện và điều khiển), luồng dữ liệu hình ảnh nặng sẽ được nén qua mạng Wifi/LAN để tránh giật lag.

Có **hai cách** để kết nối PC và Jetson:
*   **Cách 1 (Đơn giản - Direct WiFi)**: Tất cả node dùng chung `ROS_DOMAIN_ID=0` và `ROS_LOCALHOST_ONLY=0`. FastDDS tự tìm nhau qua multicast. Không cần DDS Router.
*   **Cách 2 (Có DDS Router)**: PC dùng `ROS_DOMAIN_ID=1`, Jetson dùng `ROS_DOMAIN_ID=3`. DDS Router đóng vai trò bridge WAN. Dùng khi cần kiểm soát chính xác topic nào được forward qua mạng.

> [!IMPORTANT]
> **Sai lầm phổ biến với DDS Router**: Nếu chạy `pc_gz_bridge.launch.py` trên PC mà không set `ROS_DOMAIN_ID=1`, node đó sẽ chạy trên domain 0. DDS Router Station chỉ nghe domain 1 → nó **không thấy** `/clock`, `/gimbal_camera/compressed` → không forward sang Jetson → node tracker/controller trên Jetson bị đứng im không nhận được dữ liệu nào.

---

### **BƯỚC 1: Thực hiện trên Máy PC (Host `teedee@teedee`)**

#### 1. Terminal 1: Khởi động PX4 SITL & Mô phỏng Gazebo
Mô phỏng 3D chạy trên PC để tận dụng GPU rời:
```bash
cd ~/PX4
PX4_GZ_WORLD=fractal_aruco_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
```
Trong terminal này, khi PX4 chạy ổn định, chạy thêm cầu nối địa chỉ nhận GCS từ Jetson sang:
```
mavlink start -u 14581 -o 14540 -t <CompanionPC_IP_Address> -m onboard
```
ở đây hiện tại sẽ là
```
mavlink start -u 14581 -o 14540 -t 172.20.50.44 -m onboard
```

#### 2. Terminal 2: Chạy Cầu nối (Bridges) và Nén ảnh cục bộ trên PC
Bridge này nhận clock, camera_info và ảnh từ Gazebo. Nó nén ảnh thô từ camera mô phỏng thành ảnh JPEG nén phát qua mạng:
```bash
# !! Quan trọng: phải set domain=1 để DDS Router Station nhận được topic này !!
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/PX4/examples/SITL_PrecisionLanding/ros2_ws/install/setup.bash
ros2 launch precision_landing pc_gz_bridge.launch.py
```
*   **Giải thích hoạt động**: Node `gz_image_bridge` lấy ảnh từ Gazebo đẩy vào topic nội bộ `/gimbal_camera_local` (không phát ra ngoài mạng). Node `image_compressor` nén ảnh này và phát ra topic mạng `/gimbal_camera/compressed` giúp tiết kiệm băng thông (từ 660 Mbps thô xuống còn 3 Mbps ảnh nén).

---

## Cách 1: Direct WiFi (Đơn giản, không cần DDS Router)

Tất cả terminal trên **cả PC lẫn Jetson** đều set:
```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```
Sau đó chạy các bước bình thường bên dưới. FastDDS sẽ tự tìm thấy nhau qua mạng WiFi.

---

## Cách 2: DDS Router (Kiểm soát luồng dữ liệu qua WAN)

> [!IMPORTANT]
> **Yêu cầu Domain ID theo từng máy:**
> *   **PC (`teedee`)**: `ROS_DOMAIN_ID=1` + `ROS_LOCALHOST_ONLY=1` (khớp với `StationLocal: domain=1` trong `ddsrouter_station.yaml`)
> *   **Jetson (`jb2`)**: `ROS_DOMAIN_ID=3` + `ROS_LOCALHOST_ONLY=1` (đã set sẵn trong `.bashrc`)

### **BƯỚC 0: Khởi động DDS Router trên cả hai máy (trước tiên)**

> [!NOTE]
> Cả hai file cấu hình đều nằm trên **Jetson** tại `~/DDS-Router/`:
> *   `ddsrouter_drone.yaml` → chạy trên Jetson
> *   `ddsrouter_station.yaml` → phải **copy sang PC** rồi mới chạy

**Trên Jetson**: Khởi động DDS Router drone:
```bash
cd ~/DDS-Router
source install/setup.bash
./install/ddsrouter_tool/bin/ddsrouter -c ddsrouter_drone.yaml
```

**Trên PC**:
```bash
cd ~/DDS-Router
source install/setup.bash
./install/ddsrouter_tool/bin/ddsrouter -c ddsrouter_station.yaml
```

---

### **BƯỚC 2: Cấu hình mạng & MAVROS trên Jetson (Companion `jb2@ubuntu`)**

*Lưu ý: PC và Jetson phải kết nối chung mạng LAN/Wifi, ping thông với nhau và được đặt chung một `ROS_DOMAIN_ID` trong file `.bashrc`.*

#### 1. Terminal 1: Chạy MAVROS kết nối Jetson tới PX4
*   **Nếu chạy mô phỏng SITL đa máy tính (nối qua mạng UDP)**:
    ```bash
    source /opt/ros/humble/setup.bash
    # Thay <IP_CUA_PC> bằng IP thực tế của máy PC teedee (Ví dụ: 10.70.22.100)
    ros2 launch mavros px4.launch fcu_url:=udp://:14540@<IP_CUA_PC>:14557
    ```
*   **Nếu chạy phần cứng HITL thực tế (nối với mạch Pixhawk qua cổng Serial)**:
    ```bash
    source /opt/ros/humble/setup.bash
    ros2 launch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600
    ```

#### 2. Terminal 2: Chạy bộ giải nén ảnh, Tracker và Controller trên Jetson
Khởi chạy thuật toán nhận diện và điều khiển chính:
```bash
source /opt/ros/humble/setup.bash
source ~/precision_landing_ws/install/setup.bash
ros2 launch precision_landing hitl_precland.launch.py
```
*   **Giải thích hoạt động**: 
    1.  Node `image_decompressor` giải nén ảnh từ topic mạng `/gimbal_camera/compressed` ra thành ảnh thô cục bộ `/gimbal_camera`.
    2.  Node `aruco_fractal_tracker` xử lý ảnh thô cục bộ để phát hiện tọa độ marker.
    3.  Node `offboard_precland_controller` nhận tọa độ và xuất lệnh OFFBOARD để điều khiển UAV.
    4.  Node `debug_image_compressor` nén luồng ảnh kết quả vẽ đè HUD và phát lên mạng qua topic `/landing/annotated_image/compressed`.

---

### **BƯỚC 3: Giám sát & Kích hoạt bay tự động (Thực hiện trên PC `teedee@teedee`)**

#### 1. Terminal 3: Xem ảnh camera HUD nhận diện thời gian thực (FPS cao, không lag)
Khởi chạy công cụ trực quan hóa rqt:
```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```
*   **Chọn topic**: Từ danh sách thả xuống, chọn topic **`/landing/annotated_image/compressed`** để xem vòng tròn nhận diện và thông số latency không bị trễ hình.

#### 2. Terminal 4: Nạp đường bay tự động để kích hoạt hạ cánh chính xác
```bash
source ~/PX4/examples/SITL_PrecisionLanding/ros2_ws/install/setup.bash

# Khởi chạy node telemetry upload
ros2 launch ros2_telemetry plan_upload.launch.py drone_id:=d1

# Gọi service upload mission (bao gồm lệnh COMMAND_LAND=23 để drone tự động chuyển sang chế độ hạ cánh chính xác)
ros2 service call /d1/mission_upload dib_msgs/srv/MissionUpload "{mission: [
  {command: 22, param1: 0.0, param2: 0.0, latitude: 47.397929, longitude: 8.546217, altitude: 15.0},
  {command: 16, param1: 5.0, param2: 0.0, latitude: 47.39797, longitude: 8.546322, altitude: 15.0}, 
  {command: 16, param1: 5.0, param2: 0.0, latitude: 47.3979298, longitude: 8.546217, altitude: 15.0},
  {command: 23, param1: 0.0, param2: 0.0, latitude: 47.3979298, longitude: 8.546217, altitude: 0.0} 
]}"
```

## 3.0. Hướng Dẫn Chạy Test Trên Camera Thật (Real Camera RTSP)

Để kiểm tra trực tiếp khả năng nhận diện Aruco Fractal của camera vật lý (SIYI A8 Mini hoặc bất kỳ camera IP nào) mà không làm quá tải kết nối trực tiếp của camera và chưa cần kết nối với Pixhawk, thực hiện theo các bước sau. Hệ thống hỗ trợ truyền đồng thời **luồng ảnh thô (raw)** và **luồng ảnh nhận diện (detected)** qua MediaMTX.

#### Terminal 0: Khởi động MediaMTX trên Jetson (Làm trạm phân phối luồng)
MediaMTX sẽ kết nối tới camera SIYI vật lý (`192.168.168.16`) để kéo luồng RTSP về Jetson và phát lại (relay) tại cổng nội bộ nhằm tránh gián đoạn hoặc trễ khi có nhiều kết nối đồng thời:
```bash
cd ~/mediamtx
./mediamtx mediamtx.yml
```
*Đảm bảo luồng camera đã được nhận diện và chuyển tiếp thành công tới địa chỉ cục bộ `rtsp://127.0.0.1:8554/my_camera`.*

#### Terminal 1: Khởi động Camera Publisher, Aruco Tracker và Streamer
Khởi động cụm node xử lý, tự động đẩy luồng ảnh đã được vẽ bounding box nhận diện (annotated image) ngược trở lại MediaMTX qua giao thức RTMP/RTSP:
```bash
source /opt/ros/humble/setup.bash
source ~/precision_landing_ws/install/setup.bash

# Tham số enable_mavros:=false dùng để chạy khi chưa có kết nối mạch FCU
ros2 launch precision_landing real_fractal_detect.launch.py enable_mavros:=false
```
*Lưu ý: File cấu hình `~/precision_landing_ws/src/precision_landing/config/rtsp_publisher_params.yaml` đã được thiết lập để kết nối tới `rtsp://127.0.0.1:8554/my_camera` qua MediaMTX. Node `image_to_rtsp` sẽ tự động chuyển đổi ảnh detect `/siyi/fractal_debug` và đẩy sang MediaMTX tại địa chỉ `rtsp://127.0.0.1:8554/siyi_aruco`.*

#### Theo dõi luồng video từ máy tính trạm (PC/Laptop)
Từ một máy tính khác cùng mạng WiFi/LAN, bạn có thể xem trực quan hai luồng video bằng nhiều cách:

1. **Xem qua Trình duyệt Web (WebRTC - Khuyên dùng vì độ trễ thấp nhất)**
   - **Xem trực tiếp từ Jetson** (Giả sử IP của Jetson là `<JETSON_IP>`, ví dụ `172.20.50.44`):
     - Luồng thô (Raw): `http://<JETSON_IP>:8889/my_camera/`
     - Luồng nhận dạng (Detected): `http://<JETSON_IP>:8889/siyi_aruco/`
   - **Xem qua MediaMTX trung gian của PC** (Giả sử IP của PC là `<PC_IP>`, ví dụ `172.20.50.235` và bạn đã cấu hình MediaMTX ở bước dưới):
     - Luồng thô (Raw): `http://<PC_IP>:8889/raw/`
     - Luồng nhận dạng (Detected): `http://<PC_IP>:8889/detected/`
     *(Lưu ý: Phải truy cập đúng tên đường dẫn tương ứng với tên path khai báo trong file cấu hình `.yml` của PC, ví dụ `/raw/` và `/detected/`).*

2. **Xem qua địa chỉ RTSP (sử dụng VLC Player hoặc QGroundControl)**
   - **Xem trực tiếp từ Jetson**:
     - Luồng thô (Raw): `rtsp://<JETSON_IP>:8554/my_camera`
     - Luồng nhận dạng (Detected): `rtsp://<JETSON_IP>:8554/siyi_aruco`

3. **Cấu hình MediaMTX trên máy tính trạm** (để kéo luồng tự động về PC):
   Thêm cấu hình sau vào phần cuối (`paths`) của file `mediamtx.yml` trên máy tính trạm:
   ```yaml
   paths:
     # Luồng video thô từ Jetson
     raw:
       source: rtsp://172.20.50.44:8554/my_camera
       sourceOnDemand: no
     # Luồng video có vẽ bounding box detect Aruco từ Jetson
     detected:
       source: rtsp://172.20.50.44:8554/siyi_aruco
       sourceOnDemand: no
   ```
   *(Thay đổi IP `172.20.50.44` thành IP thực tế của Jetson Orin Nano).*

#### Kiểm tra luồng tọa độ (pose) nhận diện liên tục:
```bash
source /opt/ros/humble/setup.bash
source ~/precision_landing_ws/install/setup.bash
ros2 topic echo /siyi/fractal_pose
```


---

## 4.0. Hướng Dẫn Đo Đạc Hiệu Năng Hệ Thống (CPU, GPU, VIC, NVDEC, Latency)

Thư mục `scripts` cung cấp sẵn công cụ đo đạc hiệu năng tự động là [analyze_performance.py](file:///home/jb2/precision_landing_ws/src/precision_landing/scripts/analyze_performance.py). Script này sẽ tự động thu thập các chỉ số phần cứng Jetson (qua `tegrastats`) và tính toán tần số (FPS), tỷ lệ nhận diện thành công (detection rate), sai số định vị (accuracy), độ trễ truyền dẫn (End-to-End Latency).

#### Cách chạy đo đạc:
Mở một terminal mới và thực hiện tuần tự các lệnh sau:
```bash
# 1. Chuyển vào thư mục chứa script
cd ~/precision_landing_ws/src/precision_landing/scripts/

# 2. Source môi trường để nhận diện topic ROS 2
source ~/precision_landing_ws/install/setup.bash

# 3. Đặt cấu hình kết nối mạng nội bộ (phải trùng với Terminal 1)
export ROS_LOCALHOST_ONLY=1

# 4. Cache mật khẩu sudo một lần (để script gọi tegrastats đọc chỉ số GPU/VIC)
sudo true

# 5. Chạy script phân tích hiệu năng (chạy trong 60 giây)
python3 analyze_performance.py
```

#### Kết quả đầu ra mẫu:
Script sẽ chạy thu thập dữ liệu trong 60 giây và in ra báo cáo tổng hợp dạng:
```text
==================================================
MEASUREMENT RESULTS (60 Seconds)
==================================================
Total Frames Received : 1345
Average CPU Usage    : 19.2%
Average GPU Usage    : 0.0% (GR3D)
Average VIC Usage    : 5.7% (Video Image Coprocessor)
Average NVDEC Usage  : 20.8%
Average FPS          : 22.41
Detection Rate       : 100.0%
Average Distance     : 2.06m
Accuracy (Std Dev)   : ±0.014m
Average E2E Latency  : 12.0ms
==================================================
```
*Script cũng sẽ tự động xuất ra một hàng bảng Markdown chuẩn để bạn copy trực tiếp vào báo cáo hiệu năng.*

---

## 5.0. Hướng Dẫn Chạy Full Trên Hệ Thống Phần Cứng Hoàn Toàn Không Mô Phỏng (Real Flight)

Hướng dẫn này thực hiện điều khiển bay và hạ cánh chính xác hoàn toàn trên phần cứng thật: **Drone + Mạch Pixhawk FCU + Jetson Orin/Xavier Companion Computer + Camera SIYI A8 Mini (RTSP) + MediaMTX + MAVROS + QGroundControl (PC mặt đất)**.

### Sơ đồ Kiến trúc & Kết nối Dữ liệu (Hardware Topology):
```text
[Camera SIYI A8 Mini]  --(RTSP/Ethernet)--> [MediaMTX @ Jetson]
                                                  | (RTSP Input)
[Pixhawk FCU (PX4)] <-- (Serial TELEM2 @ 921600) -- [Jetson Orin Nano]
       |                                          |-- MAVROS (Serial/UDP)
       | (WiFi/MAVLink Telemetry)                 |-- precland_container (C++ Zero-Copy)
       v                                          |   |-- RtspPublisher (GPU NVDEC)
[QGroundControl @ PC] <--(WebRTC/RTSP Viewer)-----|   |-- ArucoFractalTracker (Nested Marker)
                                                      |-- OffboardPreclandController (FSM)
                                                      |-- ImageToRtsp (RTMP Stream)
```

---

### Quy Trình Vận Hành Từng Bước Trên Hệ Thống Thật:

#### Bước 1: Khởi động MediaMTX trên Companion Computer (Jetson)
MediaMTX đóng vai trò trạm relay luồng video từ camera SIYI vật lý (`192.168.168.16`) và làm server phân phối luồng video HUD overlay qua WebRTC/RTSP.
```bash
cd ~/mediamtx
./mediamtx mediamtx.yml
```
*Đảm bảo luồng camera thô đã sẵn sàng tại `rtsp://127.0.0.1:8554/my_camera`.*

#### Bước 2: Khởi động MAVROS độc lập kết nối Pixhawk FCU
Mở một Terminal mới trên Jetson để chạy MAVROS. Chọn 1 trong 2 cách kết nối phù hợp với hệ thống của bạn:

* **Cách A: Truyền telemetry về PC mặt đất qua WiFi (Khuyên dùng)**:
  ```bash
  source /opt/ros/humble/setup.bash
  # Thay <PC_IP> bằng IP thực tế của máy tính trạm QGroundControl (Ví dụ: 192.168.1.100)
  ros2 launch mavros px4.launch \
    fcu_url:=/dev/ttyTHS1:921600 \
    gcs_url:=udp://@<PC_IP>:14550
  ```
* **Cách B: Sử dụng kết nối UDP Local (nếu chạy MAVLink bridge nội bộ) (Đang dùng trên Panther)**:
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 launch mavros px4.launch fcu_url:=udp://:14541@127.0.0.1:14540
  ```

Kiểm tra kết nối MAVROS với Pixhawk:
```bash
ros2 topic echo --once /mavros/state
```
*Kỳ vọng: `connected: true`.*

#### Bước 3: Khởi động Container Precision Landing (Vision + Controller + Streamer)
Mở một Terminal mới trên Jetson để chạy toàn bộ pipeline C++ hạ cánh chính xác qua file [real_precland.launch.py](file:///home/teedee/github_ws/precision_landing/launch/real_precland.launch.py):

```bash
source /opt/ros/humble/setup.bash
source ~/precision_landing_ws/install/setup.bash

ros2 launch precision_landing real_precland.launch.py \
  rtsp_url:=rtsp://127.0.0.1:8554/my_camera \
  flip_180:=true \
  stream_output_url:=rtmp://127.0.0.1:1935/siyi_aruco
```

#### Bước 4: Giám sát Trực quan và Điều khiển từ Máy tính Mặt đất (PC / GCS)

1. **Theo dõi luồng ảnh bám bắt có HUD Overlay (Không dùng rqt)**:
   Mở trình duyệt Web (Chrome / Firefox / Safari) trên PC hoặc thiết bị cầm tay kết nối chung mạng WiFi với Jetson (`<JETSON_IP>`):
   - **Xem luồng nhận dạng bám bắt (Annotated HUD)**: `http://<JETSON_IP>:8889/siyi_aruco/`
   - **Xem luồng camera thô (Raw Camera)**: `http://<JETSON_IP>:8889/my_camera/`
   - *(Hoặc xem qua RTSP bằng VLC Player: `rtsp://<JETSON_IP>:8554/siyi_aruco`)*

2. **Giám sát Trạng thái FSM và Tần số Setpoint**:
   - Kiểm tra FSM Controller:
     ```bash
     ros2 topic echo /offboard_precland_controller/state
     ```
   - Kiểm tra tần số setpoint gửi sang MAVROS (đạt >= 20Hz - target 30Hz):
     ```bash
     ros2 topic hz /mavros/setpoint_position/local
     ```

3. **Kích hoạt Chế độ Hạ Cánh Chính Xác**:
   - Cất cánh drone lên độ cao an toàn (Ví dụ: 8m - 12m) bằng chế độ Position / Mission.
   - Bay drone tới vùng phụ cận có đặt marker Fractal ArUco.
   - Chuyển chế độ bay sang **OFFBOARD** (hoặc gửi lệnh trigger/switch mode từ QGroundControl). Controller sẽ tự động bám đuổi marker, hạ độ cao dần theo từng tầng marker lồng nhau và đáp chính xác xuống tâm marker.

---

### Danh Mục Kiểm Tra An Toàn Trước Khi Bay Thật (Pre-flight Checklist):

> [!IMPORTANT]
> **Bắt buộc kiểm tra 4 điểm dưới đây trước khi bật công tắc OFFBOARD trên thực địa:**

1. **Kích thước Marker Thực tế (`marker_size`)**:
   - Kiểm tra tấm Fractal Marker in ra thực tế. Đo kích thước viền ngoài cùng (Level 1).
   - Nếu kích thước là `0.50m` (50cm), giữ nguyên `marker_size: 0.50` trong `config/offboard_precland_params.yaml`.
   - Nếu kích thước thực tế khác (ví dụ `0.40m`), bắt buộc sửa `marker_size` trong file YAML cho đúng kích thước thực tế để khoảng cách 3D (Z altitude) không bị tính toán sai.

2. **Chiều Lật Ảnh Camera (`flip_180`)**:
   - Mở luồng ảnh WebRTC `http://<JETSON_IP>:8889/siyi_aruco/` trên máy tính.
   - Giơ thử marker hoặc nhấc drone lên di chuyển: Đảm bảo hình ảnh hiển thị đúng chiều không bị lộn ngược đầu đuôi.

3. **Tọa độ Lệch Tâm Camera so với Tâm Drone (`camera_offset`)**:
   - Kiểm tra thông số `camera_offset_x`, `camera_offset_y`, `camera_offset_z` trong `config/offboard_precland_params.yaml` khớp với vị trí gá lắp thực tế của ống kính camera so với tâm trọng tâm (CG) của drone.

4. **Công Tắc Ngắt Chế Độ An Toàn Trên Tay Điều Khiển (RC Kill / Manual Override Switch)**:
   - Gán sẵn công tắc chuyển đổi chế độ bay (Flight Mode Switch) trên Tay điều khiển (RC Transmitter) sang chế độ **Position Hold** hoặc **Altitude / Stabilized**.
   - Trong quá trình hạ cánh tự động nếu có chướng ngại vật hoặc gió quá lớn, phi công chỉ cần gạt công tắc RC sang Position Mode là tay điều khiển lấy lại quyền lái ngay lập tức.

---

## Phụ lục PL1: Hướng dẫn Quản lý Điện năng & Hiệu năng trên Jetson (nvpmodel & jetson_clocks)

Để tối ưu hóa phần cứng Jetson Orin Nano cho các tác vụ thời gian thực (như Precision Landing) hoặc đưa về chế độ mặc định để tiết kiệm năng lượng và tăng tuổi thọ quạt tản nhiệt, bạn thực hiện theo hướng dẫn dưới đây.

### 1. Công cụ nvpmodel (Quản lý các Chế độ Điện năng)
`nvpmodel` quản lý các profile giới hạn công suất tiêu thụ (ví dụ: MAXN, 15W, 10W) bằng cách giới hạn số nhân CPU online và dải tần số (xung nhịp) tối đa của CPU/GPU/RAM.

- **Xem chế độ hiện tại và danh sách tất cả các chế độ hỗ trợ:**
  ```bash
  sudo nvpmodel -q --verbose
  ```
- **Bật chế độ công suất tối đa (MAXN - Mode 0):**
  *(Kích hoạt toàn bộ nhân CPU và GPU chạy ở giới hạn xung nhịp cao nhất. Dùng khi bay thử nghiệm hoặc xử lý nặng).*
  ```bash
  sudo nvpmodel -m 0
  ```
- **Quay về chế độ mặc định tiết kiệm điện (15W - Mode 1):**
  *(Đưa Jetson về trạng thái tiêu thụ điện năng 15W mặc định khi xuất xưởng, giúp mát máy và tiết kiệm pin).*
  ```bash
  sudo nvpmodel -m 1
  ```
- *Lưu ý:* Cấu hình của `nvpmodel` được **tự động lưu vĩnh viễn** và tự áp dụng lại sau khi reboot.

### 2. Công cụ jetson_clocks (Khóa cứng Xung nhịp & Tốc độ Quạt)
`jetson_clocks` là một script tiện ích của NVIDIA dùng để vô hiệu hóa tính năng tự động điều tốc (DVFS) và ép cứng toàn bộ các thành phần (CPU, GPU, RAM) chạy ở tần số tối đa cho phép của chế độ `nvpmodel` đang hoạt động, đồng thời ép quạt quay 100% để đảm bảo làm mát.

- **Xem thông số xung nhịp và tốc độ quạt thực tế hiện tại:**
  ```bash
  sudo jetson_clocks --show
  ```
- **Ép cứng hiệu năng tối đa (Bật):**
  ```bash
  sudo jetson_clocks
  ```
- **Tắt chế độ ép xung và quay về tự động điều tốc:**
  ```bash
  sudo jetson_clocks --restore
  ```
  *(Hoặc đơn giản là **khởi động lại máy (reboot)**, các thiết lập của `jetson_clocks` sẽ tự động bị xóa bỏ).*

### 3. Khuyên dùng cho Bay Thử nghiệm & Vận hành thực tế
- Do thuật toán của chúng ta đã được tối ưu hóa cực kỳ sâu (Zero-copy IPC, GStreamer NVDEC/VIC giải mã phần cứng), tải CPU tiêu thụ rất thấp (chỉ khoảng **12% - 18%**). 
- Để bảo vệ tuổi thọ quạt tản nhiệt và tránh hao pin drone ngoài ý muốn, **khuyến cáo không cần chạy `jetson_clocks`** trong vận hành thực tế. Bạn chỉ cần bật `sudo nvpmodel -m 1` và để hệ điều hành tự động tăng giảm xung nhịp linh hoạt.

### 4. Tự động kích hoạt hiệu năng tối đa khi khởi động (Permanent Boot Config)
Nếu bạn muốn Jetson luôn tự động chạy ở hiệu năng cao nhất (MAXN - Mode 0) và khóa cứng xung nhịp `jetson_clocks` mỗi khi cấp nguồn hoặc khởi động lại mà không cần chạy thủ công, hãy thực hiện cấu hình vĩnh viễn như sau:

#### Bước 4.1: Cố định nvpmodel MAXN
Chạy lệnh cấu hình nvpmodel sang mode 0 một lần duy nhất. Hệ thống sẽ tự động lưu lại trạng thái này vĩnh viễn (lưu vào `/var/lib/nvpmodel/status` và được nạp lại thông qua `nvpmodel.service` lúc khởi động):
```bash
sudo nvpmodel -m 0
```

#### Bước 4.2: Tạo systemd Service để tự khởi chạy jetson_clocks
Vì cấu hình của `jetson_clocks` sẽ bị mất sau khi reboot, chúng ta tạo một dịch vụ hệ thống (systemd) để chạy lệnh này tự động sau khi `nvpmodel` đã khởi động:

1. **Tạo file cấu hình dịch vụ `/etc/systemd/system/jetson_clocks.service`:**
   ```bash
   sudo bash -c 'cat <<EOF > /etc/systemd/system/jetson_clocks.service
   [Unit]
   Description=Maximize Jetson Clocks
   After=nvpmodel.service
   Requires=nvpmodel.service

   [Service]
   Type=oneshot
   ExecStart=/usr/bin/jetson_clocks
   RemainAfterExit=yes

   [Install]
   WantedBy=multi-user.target
   EOF'
   ```

2. **Phân quyền và kích hoạt dịch vụ chạy cùng hệ thống:**
   ```bash
   sudo chmod 644 /etc/systemd/system/jetson_clocks.service
   sudo systemctl daemon-reload
   sudo systemctl enable jetson_clocks.service
   ```

3. **Khởi chạy thử nghiệm và kiểm tra trạng thái dịch vụ:**
   ```bash
   sudo systemctl start jetson_clocks.service
   sudo systemctl status jetson_clocks.service
   ```
   *(Trạng thái hiển thị `Active: active (exited)` màu xanh nghĩa là dịch vụ đã hoạt động thành công và sẽ tự nạp lại khi có điện/reboot).*

#### Bước 4.3: Hủy kích hoạt và xóa bỏ Service (Khi không muốn dùng nữa)
Nếu muốn tắt tính năng tự động ép xung và xóa hoàn toàn dịch vụ này khỏi hệ thống, bạn chạy các lệnh sau:
1. **Dừng và hủy kích hoạt dịch vụ chạy lúc khởi động:**
   ```bash
   sudo systemctl stop jetson_clocks.service
   sudo systemctl disable jetson_clocks.service
   ```
2. **Xóa file service và reload lại cấu hình hệ thống:**
   ```bash
   sudo rm /etc/systemd/system/jetson_clocks.service
   sudo systemctl daemon-reload
   sudo systemctl reset-failed
   ```
3. **Đưa xung nhịp và quạt tản nhiệt về chế độ tự động điều tốc ngay lập tức (không cần reboot):**
   ```bash
   sudo jetson_clocks --restore
   ```

---

## Phụ lục PL2: Hướng dẫn Can thiệp Thủ công & Giành lại Quyền Điều khiển (Manual RC Override)

Trong quá trình hạ cánh tự động (chế độ **OFFBOARD**), phi công có thể chiếm lại quyền điều khiển thủ công từ tay cầm RC bất kỳ lúc nào bằng 2 cơ chế sau:

### 1. Cơ chế 1: Gạt công tắc chuyển Chế độ bay (Flight Mode Switch)
* **Cách thực hiện:** Phi công gạt công tắc chuyển chế độ bay trên tay cầm RC sang **`Position Mode` (POSCTL)**, **`Altitude Mode` (ALTCTL)** hoặc **`Stabilized`**.
* **Tác dụng:** Lập tức ngắt chế độ `OFFBOARD`. Firmware PX4 trao toàn quyền điều khiển thủ công lại cho tay cầm RC.
* **Độ ưu tiên:** Tuyệt đối (phần cứng & firmware PX4 luôn ưu tiên công tắc RC hơn lệnh từ ROS 2).

### 2. Cơ chế 2: Đẩy cần gạt điều khiển (RC Stick Override)
Mặc định trong PX4, tính năng tự động ngắt OFFBOARD khi đẩy cần gạt bị tắt (`COM_RC_OVERRIDE` mặc định `= 1`). Để cho phép phi công đẩy nhẹ cần gạt là tự ngắt OFFBOARD:

#### Cấu hình tham số trên QGroundControl (QGC):
1. Kết nối drone với **QGroundControl**.
2. Vào **Vehicle Setup** $\rightarrow$ **Parameters**.
3. Tìm kiếm tham số: **`COM_RC_OVERRIDE`**.
4. Chuyển giá trị từ `1` sang **`3`** (tương ứng tích chọn cả `Auto modes` và `Offboard mode`).
5. Reboot hoặc Lưu tham số vào PX4.

#### Tác dụng & Hướng dẫn sử dụng:
* **Tác dụng:** Khi `COM_RC_OVERRIDE = 3`, nếu drone đang tự động hạ cánh ở chế độ `OFFBOARD`, phi công chỉ cần đẩy nhẹ cần gạt (Roll, Pitch, Yaw hoặc Throttle) vượt quá $30\%$, PX4 sẽ tự động ngắt `OFFBOARD` và chuyển ngay sang chế độ **`Position Mode`**.
* **Hướng dẫn điều khiển:** Sau khi ngắt thành công, phi công sử dụng cần gạt tay cầm RC để tự bay đứng yên, di chuyển hoặc hạ cánh bằng tay an toàn.

---

## Phụ lục PL3: Xử lý Vấn đề Rãnh đứt (Gap/Seam) trên Landing Pad bằng Tiền Xử Lý Hình Thái Học (Morphology Preprocessing)

### 1. Mô tả Vấn đề
Khi dán **Fractal ArUco Marker** lên nắp Trạm / Landing Box có cơ chế mở 4 cánh cửa, khe hở rãnh màu trắng (hình chữ thập `+`) cắt ngang đường viền đen outer border của **Marker lớn nhất (ID 0)**.
* **Hậu quả:** Đường viền đen bị đứt đoạn thành 4 khối L riêng biệt, khiến thuật toán quét contour (`cv::findContours`) trong OpenCV / ArUco không thể nhận diện được hình vuông outer border của Marker ID 0 khi UAV ở độ cao lớn (Z > 3m).

### 2. Giải pháp Kỹ thuật (Morphology Opening)
Node `aruco_fractal_tracker` tích hợp bộ lọc **Hình thái học (Morphology Opening)** trên ảnh xám (Grayscale) trước khi đưa ảnh vào ArUco detector:
* **Nguyên lý:** Trong ảnh xám OpenCV (Màu đen = 0, Màu trắng = 255), phép toán `cv::MORPH_OPEN` (`cv::erode` làm nở màu đen `0`, sau đó `cv::dilate` thu nhỏ lại) giúp vùng màu đen ở 2 bên khe hở "tràn" qua lấp đầy rãnh trắng mà không làm thay đổi tổng thể kích thước marker.
* **Kết quả:** Nối liền đường viền đen đứt đoạn thành 1 contour khép kín liên tục, khôi phục khả năng detect Marker ID 0 từ xa/trên cao.

### 3. Hướng dẫn Chạy Test & Tinh chỉnh trên Webcam Laptop / Camera Thực
Hệ thống cung cấp file launch tiện lợi kèm theo các tham số tinh chỉnh kernel làm mờ trực tiếp:

#### Step 1: Khởi chạy Test với Webcam
```bash
source /opt/ros/humble/setup.bash
source ~/precision_landing_ws/install/setup.bash

# Khởi chạy webcam kèm bộ lọc lấp rãnh (mặc định kernel size = 7)
ros2 launch precision_landing webcam_detect.launch.py morphology_kernel_size:=7
```

#### Step 2: Tinh chỉnh Kích thước Kernel theo Rãnh hở thực tế
Nếu rãnh hở trên nắp pad chiếm bề rộng nhiều pixel hơn trên camera:
* Tăng `morphology_kernel_size` lên `9`, `11` hoặc `15`:
  ```bash
  ros2 launch precision_landing webcam_detect.launch.py morphology_kernel_size:=11
  ```
  *(Lưu ý: Không đặt kernel quá lớn > 31 vì sẽ làm nhòe mất ma trận bit mã hóa bên trong marker).*

#### Step 3: Kiểm tra Trực quan Ảnh Sau Xử lý (Debug Topic)
Mở cửa sổ xem ảnh trực tiếp sau khi qua bộ lọc lấp rãnh tại topic `/siyi/preprocessed_image`:
```bash
ros2 run rqt_image_view rqt_image_view /siyi/preprocessed_image
```
*(Nếu thấy đường viền đen đã nối liền rãnh trắng trên cửa sổ `/siyi/preprocessed_image`, Marker ID 0 sẽ được nhận diện thành công trên cửa sổ `/siyi/fractal_debug`).*



