#include "precision_landing/modules/target_tracker.hpp"
#include "precision_landing/modules/guidance_controller.hpp"
#include <cmath>
#include <algorithm>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace precision_landing
{

TargetTracker::TargetTracker(rclcpp::Node * node, tf2_ros::Buffer * tf_buffer)
: node_(node), tf_buffer_(tf_buffer)
{
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
}

void TargetTracker::publish_static_transform(const std::string & camera_frame, const CameraParams & cam_params)
{
  if (tf_static_published_) {
    return;
  }
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node_->get_clock()->now();
  t.header.frame_id = "base_link";
  t.child_frame_id = camera_frame;
  t.transform.translation.x = cam_params.camera_offset_x;
  t.transform.translation.y = cam_params.camera_offset_y;
  t.transform.translation.z = cam_params.camera_offset_z;

  tf2::Quaternion q;
  q.setRPY(cam_params.camera_mount_roll * M_PI / 180.0,
           cam_params.camera_mount_pitch * M_PI / 180.0,
           cam_params.camera_mount_yaw * M_PI / 180.0);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  tf_static_broadcaster_->sendTransform(t);
  tf_static_published_ = true;
  RCLCPP_INFO(node_->get_logger(), "Published static transform base_link -> %s (RPY=%.1f, %.1f, %.1f)",
              camera_frame.c_str(), cam_params.camera_mount_roll, cam_params.camera_mount_pitch, cam_params.camera_mount_yaw);
}

Vector3 TargetTracker::camera_to_enu(double cam_x, double cam_y, const Quaternion & q_att, const CameraParams & cam_params)
{
  if (cam_params.camera_yaw_frame == "local") {
    return Vector3{cam_x, cam_y, 0.0};
  }
  double yaw = GuidanceController::get_yaw(q_att);
  double eb = cam_x;
  double nb = cam_y;
  double xb = nb + cam_params.camera_offset_x;
  double yb = -eb + cam_params.camera_offset_y;
  double c = std::cos(yaw);
  double s = std::sin(yaw);
  return Vector3{
    xb * c - yb * s,
    xb * s + yb * c,
    0.0
  };
}

std::tuple<Vector3, Quaternion> TargetTracker::get_historical_state(
  double time, const std::deque<std::tuple<double, Vector3, Quaternion>> & history,
  const Vector3 & current_pos, const Quaternion & current_q)
{
  if (history.empty()) {
    return {current_pos, current_q};
  }

  auto it = std::lower_bound(history.begin(), history.end(), time,
    [](const std::tuple<double, Vector3, Quaternion>& a, double val) {
      return std::get<0>(a) < val;
    }
  );

  if (it == history.end()) {
    return {std::get<1>(history.back()), std::get<2>(history.back())};
  }
  if (it == history.begin()) {
    return {std::get<1>(history.front()), std::get<2>(history.front())};
  }

  auto prev_it = std::prev(it);
  double diff1 = std::abs(std::get<0>(*it) - time);
  double diff2 = std::abs(std::get<0>(*prev_it) - time);

  if (diff1 < diff2) {
    return {std::get<1>(*it), std::get<2>(*it)};
  } else {
    return {std::get<1>(*prev_it), std::get<2>(*prev_it)};
  }
}

void TargetTracker::process_target_msg(
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
  double sp_yaw)
{
  tracking_count_++;

  geometry_msgs::msg::PoseStamped msg_map;
  bool tf_ok = false;
  double world_yaw_sample = 0.0;
  double abs_x = 0.0;
  double abs_y = 0.0;
  double raw_pad_z = 0.0;
  Quaternion h_q{1.0, 0.0, 0.0, 0.0};

  try {
    geometry_msgs::msg::PoseStamped msg_zero_time = *msg;
    msg_zero_time.header.stamp = rclcpp::Time(0);
    msg_map = tf_buffer_->transform(msg_zero_time, "map", tf2::durationFromSec(0.05));
    abs_x = msg_map.pose.position.x;
    abs_y = msg_map.pose.position.y;
    raw_pad_z = msg_map.pose.position.z;

    Quaternion q_tag_world{
      msg_map.pose.orientation.w,
      msg_map.pose.orientation.x,
      msg_map.pose.orientation.y,
      msg_map.pose.orientation.z
    };
    world_yaw_sample = GuidanceController::get_yaw(q_tag_world);
    tf_ok = true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "TF2 Transform to map failed: %s. Using manual fallback.", ex.what());

    double tvec_x = msg->pose.position.x;
    double tvec_y = msg->pose.position.y;
    double tvec_z = msg->pose.position.z;
    double cam_x = tvec_x; // camera sign transform if needed
    double cam_y = tvec_y;

    double t_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    Vector3 h_pos;
    std::tie(h_pos, h_q) = get_historical_state(t_time, history, pos_enu, q_att);
    Vector3 rel = CameraParams{}.camera_yaw_frame == "local" ? Vector3{cam_x, cam_y, 0.0} : Vector3{cam_x, cam_y, 0.0};

    abs_x = h_pos.x + rel.x;
    abs_y = h_pos.y + rel.y;
    raw_pad_z = h_pos.z - tvec_z;

    Quaternion q_tag_cam{
      msg->pose.orientation.w,
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z
    };
    Quaternion q_cam_body{0.0, 0.7071067811865475, -0.7071067811865475, 0.0};
    Quaternion q_tag_world = GuidanceController::quaternion_multiply(GuidanceController::quaternion_multiply(h_q, q_cam_body), q_tag_cam);
    world_yaw_sample = GuidanceController::get_yaw(q_tag_world);
  }

  // Calculate error relative to drone position
  double rel_x = abs_x - pos_enu.x;
  double rel_y = abs_y - pos_enu.y;
  double rn = std::sqrt(rel_x*rel_x + rel_y*rel_y);

  // Dynamic reject check (caller can verify against get_reject_r)
  double max_pad_z_step = 0.02; // m/tick
  double pad_z_target = ema_alpha_pad_ * raw_pad_z + (1.0 - ema_alpha_pad_) * virtual_pad_z_;
  double d = pad_z_target - virtual_pad_z_;
  virtual_pad_z_ += std::clamp(d, -max_pad_z_step, max_pad_z_step);

  target_enu_ = {abs_x, abs_y};
  target_rel_norm_ = rn;
  last_pose_time_ = node_->get_clock()->now().nanoseconds() * 1e-9;

  if (align_yaw_to_tag) {
    double current_alt = std::max(0.0, pos_enu.z - virtual_pad_z_);
    double target_lock_alt = (yaw_lock_stage <= 1) ? yaw_lock_alt : yaw_lock_alt_2;
    if (current_state == PrecLandState::DESCEND_ABOVE_TARGET && !yaw_locked && current_alt <= target_lock_alt) {
      yaw_lock_buf.push_back(world_yaw_sample);
      if (static_cast<int>(yaw_lock_buf.size()) >= yaw_lock_samples) {
        if (!yaw_lock_buf.empty()) {
          tag_yaw_abs = GuidanceController::circular_mean(yaw_lock_buf);
          yaw_locked = true;
          RCLCPP_INFO(
            node_->get_logger(),
            "[YAW-LOCK] latched target=%.1f deg from %d samples at %.1fm",
            tag_yaw_abs.value() * 180.0 / M_PI, (int)yaw_lock_buf.size(), current_alt
          );
        }
      }
    }

    if (tracking_count_ % 15 == 0) {
      double body_yaw = tf_ok ? GuidanceController::get_yaw(q_att) : GuidanceController::get_yaw(h_q);
      RCLCPP_INFO(
        node_->get_logger(),
        "[YAW-3D] alt=%.1fm stage=%d body=%.1f deg | sample=%.1f deg | locked=%s target=%.1f deg | buf=%d/%d | sp=%.1f deg",
        pos_enu.z, yaw_lock_stage, body_yaw * 180.0 / M_PI, world_yaw_sample * 180.0 / M_PI,
        yaw_locked ? "true" : "false", tag_yaw_abs.has_value() ? tag_yaw_abs.value() * 180.0 / M_PI : 0.0,
        (int)yaw_lock_buf.size(), yaw_lock_samples, sp_yaw * 180.0 / M_PI
      );
    }
  }
}

double TargetTracker::get_alt(double pos_z)
{
  return std::max(0.0, pos_z - virtual_pad_z_);
}

double TargetTracker::get_blend(double pos_z, double high_alt, double low_alt)
{
  double span = std::max(0.1, high_alt - low_alt);
  return std::min(1.0, std::max(0.0, (get_alt(pos_z) - low_alt) / span));
}

double TargetTracker::get_alpha(double pos_z, const GateParams & p)
{
  double t = get_blend(pos_z, p.high_alt, p.low_alt);
  return p.alpha_low * (1.0 - t) + p.alpha_high * t;
}

double TargetTracker::get_servo_gain(double pos_z, const GateParams & p)
{
  double t = get_blend(pos_z, p.high_alt, p.low_alt);
  return p.servo_gain_low * (1.0 - t) + p.servo_gain_high * t;
}

double TargetTracker::get_align_r(double pos_z, const GateParams & p)
{
  double v = p.align_r_bias + p.align_r_gain * get_alt(pos_z);
  return std::min(p.align_r_max, std::max(p.align_r_min, v));
}

double TargetTracker::get_descent_r(double pos_z, const GateParams & p)
{
  double v = p.desc_r_bias + p.desc_r_gain * get_alt(pos_z);
  return std::min(p.desc_r_max, std::max(p.desc_r_min, v));
}

double TargetTracker::get_reject_r(double pos_z, const GateParams & p)
{
  double v = p.reject_r_gain * get_alt(pos_z);
  return std::min(p.reject_r_max, std::max(p.reject_r_min, v));
}

bool TargetTracker::is_target_fresh(double now_sec, double target_timeout)
{
  return (now_sec - last_pose_time_) < target_timeout && last_pose_time_ > 0;
}

}  // namespace precision_landing
