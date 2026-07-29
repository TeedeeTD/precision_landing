#ifndef PRECISION_LANDING__MODULES__TARGET_TRACKER_HPP_
#define PRECISION_LANDING__MODULES__TARGET_TRACKER_HPP_

#include <memory>
#include <deque>
#include <tuple>
#include <optional>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "precision_landing/types.hpp"

namespace precision_landing
{

struct GateParams
{
  double high_alt{10.0};
  double low_alt{2.0};
  double alpha_high{0.8};
  double alpha_low{0.2};
  double servo_gain_high{0.5};
  double servo_gain_low{0.2};
  double align_r_min{0.3};
  double align_r_max{1.5};
  double align_r_gain{0.1};
  double align_r_bias{0.2};
  double desc_r_min{0.3};
  double desc_r_max{1.5};
  double desc_r_gain{0.1};
  double desc_r_bias{0.2};
  double reject_r_min{0.5};
  double reject_r_max{3.0};
  double reject_r_gain{0.3};
};

struct CameraParams
{
  double camera_x_to_body_east_sign{1.0};
  double camera_y_to_body_north_sign{1.0};
  std::string camera_yaw_frame{"body"};
  double camera_offset_x{0.0};
  double camera_offset_y{0.0};
  double camera_offset_z{0.0};
  double camera_mount_roll{0.0};
  double camera_mount_pitch{0.0};
  double camera_mount_yaw{0.0};
};

class TargetTracker
{
public:
  TargetTracker(rclcpp::Node * node, tf2_ros::Buffer * tf_buffer);
  ~TargetTracker() = default;

  void process_target_msg(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg,
    const Vector3 & pos_enu,
    const Quaternion & q_att,
    const std::deque<std::tuple<double, Vector3, Quaternion>> & history,
    bool align_yaw_to_tag,
    PrecLandState current_state,
    int yaw_lock_samples,
    double yaw_lock_alt,
    double yaw_lock_alt_2,
    int & yaw_lock_stage,
    bool & yaw_locked,
    std::vector<double> & yaw_lock_buf,
    std::optional<double> & tag_yaw_abs,
    double sp_yaw);

  void publish_static_transform(const std::string & camera_frame, const CameraParams & cam_params);

  Vector3 camera_to_enu(double cam_x, double cam_y, const Quaternion & q_att, const CameraParams & cam_params);
  std::tuple<Vector3, Quaternion> get_historical_state(
    double time, const std::deque<std::tuple<double, Vector3, Quaternion>> & history,
    const Vector3 & current_pos, const Quaternion & current_q);

  // Dynamic Gate Calculations
  double get_alt(double pos_z);
  double get_blend(double pos_z, double high_alt, double low_alt);
  double get_alpha(double pos_z, const GateParams & p);
  double get_servo_gain(double pos_z, const GateParams & p);
  double get_align_r(double pos_z, const GateParams & p);
  double get_descent_r(double pos_z, const GateParams & p);
  double get_reject_r(double pos_z, const GateParams & p);
  bool is_target_fresh(double now_sec, double target_timeout);

  // Getters & Setters
  std::optional<std::tuple<double, double>> get_target_enu() const { return target_enu_; }
  std::optional<std::tuple<double, double>> get_target_enu_filtered() const { return target_enu_filtered_; }
  void set_target_enu_filtered(const std::optional<std::tuple<double, double>> & val) { target_enu_filtered_ = val; }
  double get_target_rel_norm() const { return target_rel_norm_; }
  double get_last_pose_time() const { return last_pose_time_; }
  int get_tracking_count() const { return tracking_count_; }
  void reset_tracking_count() { tracking_count_ = 0; }
  double get_virtual_pad_z() const { return virtual_pad_z_; }
  void reset_virtual_pad_z() { virtual_pad_z_ = 0.0; }

private:
  rclcpp::Node * node_;
  tf2_ros::Buffer * tf_buffer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  bool tf_static_published_{false};

  std::optional<std::tuple<double, double>> target_enu_;
  std::optional<std::tuple<double, double>> target_enu_filtered_;
  double target_rel_norm_{9999.0};
  double last_pose_time_{0.0};
  int tracking_count_{0};
  double virtual_pad_z_{0.0};
  double ema_alpha_pad_{0.2};
};

}  // namespace precision_landing

#endif  // PRECISION_LANDING__MODULES__TARGET_TRACKER_HPP_
