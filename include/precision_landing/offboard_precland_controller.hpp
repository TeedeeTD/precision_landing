#ifndef PRECISION_LANDING__OFFBOARD_PRECLAND_CONTROLLER_HPP_
#define PRECISION_LANDING__OFFBOARD_PRECLAND_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <tuple>
#include <optional>
#include <rclcpp/rclcpp.hpp>

#include "precision_landing/types.hpp"
#include "precision_landing/modules/vehicle_interface.hpp"
#include "precision_landing/modules/target_tracker.hpp"
#include "precision_landing/modules/precision_land_fsm.hpp"
#include "precision_landing/modules/guidance_controller.hpp"
#include "precision_landing/modules/gimbal_controller.hpp"

namespace precision_landing
{

class OffboardPreclandController : public rclcpp::Node
{
public:
  explicit OffboardPreclandController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~OffboardPreclandController() = default;

private:
  // --- ROS 2 Callbacks ---
  void on_pos(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void on_state(const mavros_msgs::msg::State::SharedPtr msg);
  void on_ext_state(const mavros_msgs::msg::ExtendedState::SharedPtr msg);
  void on_waypoints(const mavros_msgs::msg::WaypointList::SharedPtr msg);
  void on_target(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // --- Main Loop and FSM Tick ---
  void control_loop();
  void gimbal_tick();
  void query_px4_params();

  // --- FSM State Handlers ---
  void st_idle();
  void st_start();
  void st_horizontal_approach();
  void st_descend_above_target();
  void st_final_approach();
  void st_search();
  void st_target_lost();
  void st_fallback();
  void st_done();

  // --- Helper Delegates ---
  void transition(PrecLandState new_state);
  double now_sec();

  // --- Parameter Structs ---
  CameraParams cam_params_;
  GateParams gate_params_;
  GuidanceParams guidance_params_;

  // Tunable Parameters
  std::string target_topic_;
  std::string target_pose_topic_;
  bool align_yaw_to_tag_;
  int land_mode_;
  double abort_alt_param_;
  double final_alt_param_;
  int yaw_lock_samples_;
  double yaw_lock_alt_;
  double yaw_lock_alt_2_;

  int ctrl_hz_;
  double target_timeout_;
  int tracking_confirm_;
  int align_confirm_;
  double align_timeout_;
  double search_timeout_;
  int max_search_;
  double descent_rate_;
  double fappr_alt_;
  double hacc_rad_;
  double max_align_step_;
  double max_descent_step_;

  double target_loss_grace_;
  double descent_loss_hold_;
  double low_alt_max_err_;
  double search_alt_;
  double search_alt_max_;

  double final_approach_timeout_;
  double final_descent_rate_;
  double final_align_step_;
  double yaw_lock_timeout_;
  int yaw_lock_min_samples_;

  // --- State Variables ---
  Vector3 sp_enu_{0.0, 0.0, 0.0};
  double held_yaw_{0.0};
  std::optional<double> tag_yaw_abs_;
  bool yaw_locked_{false};
  std::vector<double> yaw_lock_buf_;
  bool yaw_realign_complete_{false};
  int realign_cnt_{0};
  int yaw_lock_stage_{0};
  double last_loop_run_time_{0.0};
  double final_x_{0.0};
  double final_y_{0.0};

  bool disarm_requested_{false};
  double disarm_attempt_time_{0.0};
  double disarm_attempt_time_first_{0.0};
  bool auto_land_fallback_sent_{false};

  double final_approach_entry_z_{0.0};

  int centered_count_{0};
  int descent_drift_count_{0};
  double descent_z_sp_{10.0};
  int target_counter_{0};
  int search_cnt_{0};
  double approach_alt_{10.0};
  double start_z_sp_{10.0};
  std::optional<Vector3> land_hold_pos_;
  bool offboard_activated_{false};

  // --- 5 Refactored Sub-Modules ---
  std::shared_ptr<VehicleInterface> vehicle_;
  std::unique_ptr<TargetTracker> target_tracker_;
  std::unique_ptr<PrecisionLandFSM> fsm_;
  std::unique_ptr<GuidanceController> guidance_;
  std::unique_ptr<GimbalController> gimbal_;

  // --- Subscriptions ---
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_target_;

  // --- TF2 ---
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // --- Timers ---
  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::TimerBase::SharedPtr gimbal_timer_;
  rclcpp::TimerBase::SharedPtr param_timer_;
};

}  // namespace precision_landing

#endif  // PRECISION_LANDING__OFFBOARD_PRECLAND_CONTROLLER_HPP_
