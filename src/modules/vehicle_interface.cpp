#include "precision_landing/modules/vehicle_interface.hpp"

namespace precision_landing
{

VehicleInterface::VehicleInterface(rclcpp::Node * node)
: node_(node)
{
  pub_sp_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10);
  pub_sp_raw_ = node_->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  set_mode_client_ = node_->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
  arm_client_ = node_->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  cmd_client_ = node_->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");
  param_get_client_ = node_->create_client<mavros_msgs::srv::ParamGet>("/mavros/param/get");
  param_set_client_ = node_->create_client<mavros_msgs::srv::ParamSet>("/mavros/param/set");
  wp_pull_client_ = node_->create_client<mavros_msgs::srv::WaypointPull>("/mavros/mission/pull");
}

void VehicleInterface::init_ros_interfaces(
  rclcpp::QoS pose_qos, rclcpp::QoS state_qos,
  std::function<void(const geometry_msgs::msg::PoseStamped::SharedPtr)> pos_cb,
  std::function<void(const mavros_msgs::msg::State::SharedPtr)> state_cb,
  std::function<void(const mavros_msgs::msg::ExtendedState::SharedPtr)> ext_state_cb,
  std::function<void(const mavros_msgs::msg::WaypointList::SharedPtr)> wp_cb)
{
  sub_pos_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", pose_qos, pos_cb);

  sub_state_ = node_->create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", state_qos, state_cb);

  sub_ext_state_ = node_->create_subscription<mavros_msgs::msg::ExtendedState>(
    "/mavros/extended_state", state_qos, ext_state_cb);

  sub_waypoints_ = node_->create_subscription<mavros_msgs::msg::WaypointList>(
    "/mavros/mission/waypoints", state_qos, [this, wp_cb](const mavros_msgs::msg::WaypointList::SharedPtr msg) {
      waypoints_ = msg->waypoints;
      current_wp_seq_ = msg->current_seq;
      wp_cb(msg);
    });
}

void VehicleInterface::push_history(double stamp, const Vector3 & pos, const Quaternion & q)
{
  history_.push_back({stamp, pos, q});
  if (history_.size() > 150) {
    history_.pop_front();
  }

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node_->get_clock()->now();
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  t.transform.translation.x = pos.x;
  t.transform.translation.y = pos.y;
  t.transform.translation.z = pos.z;
  t.transform.rotation.w = q.w;
  t.transform.rotation.x = q.x;
  t.transform.rotation.y = q.y;
  t.transform.rotation.z = q.z;
  tf_broadcaster_->sendTransform(t);
}

void VehicleInterface::pull_waypoints_immediately()
{
  if (wp_pull_client_->service_is_ready()) {
    auto req = std::make_shared<mavros_msgs::srv::WaypointPull::Request>();
    auto cb = [this](rclcpp::Client<mavros_msgs::srv::WaypointPull>::SharedFuture future) {
      try {
        auto res = future.get();
        if (res->success) {
          RCLCPP_INFO(node_->get_logger(), "Successfully pulled waypoints, received=%u", res->wp_received);
        } else {
          RCLCPP_WARN(node_->get_logger(), "Waypoint pull failed");
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(node_->get_logger(), "Waypoint pull service call failed: %s", e.what());
      }
    };
    wp_pull_client_->async_send_request(req, cb);
    RCLCPP_INFO(node_->get_logger(), "Landing phase started — pulling waypoints immediately");
  }
}

void VehicleInterface::set_mode(const std::string & mode, bool & offboard_activated)
{
  if (set_mode_client_->service_is_ready()) {
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = mode;

    auto cb = [this, mode, &offboard_activated](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
      try {
        auto res = future.get();
        if (res->mode_sent) {
          RCLCPP_INFO(node_->get_logger(), "Set mode %s succeeded", mode.c_str());
        } else {
          RCLCPP_WARN(node_->get_logger(), "Set mode %s failed (mode_sent = false)", mode.c_str());
          if (mode == "OFFBOARD") {
            offboard_activated = false; // allow retry
          }
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(node_->get_logger(), "Set mode %s service call failed: %s", mode.c_str(), e.what());
        if (mode == "OFFBOARD") {
          offboard_activated = false; // allow retry
        }
      }
    };
    set_mode_client_->async_send_request(req, cb);
  }
}

void VehicleInterface::send_command(uint16_t command, float p1, float p2, float p3, float p4, float p5, float p6, float p7)
{
  if (cmd_client_->service_is_ready()) {
    auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    req->command = command;
    req->param1 = p1;
    req->param2 = p2;
    req->param3 = p3;
    req->param4 = p4;
    req->param5 = p5;
    req->param6 = p6;
    req->param7 = p7;
    cmd_client_->async_send_request(req);
  }
}

void VehicleInterface::disarm(double & disarm_attempt_time)
{
  RCLCPP_INFO(node_->get_logger(), "Sending force-disarm (MAV_CMD 400, magic=21196)");

  if (cmd_client_->service_is_ready()) {
    auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    req->command = 400;
    req->param1 = 0.0f;
    req->param2 = 21196.0f;

    auto cb = [this](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future) {
      try {
        auto res = future.get();
        if (!res->success || res->result != 0 /* MAV_RESULT_ACCEPTED */) {
          RCLCPP_WARN(node_->get_logger(),
            "Force-disarm command NOT accepted (success=%d, result=%d) — will retry",
            res->success, res->result);
        } else {
          RCLCPP_INFO(node_->get_logger(), "Force-disarm command ACCEPTED by PX4");
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(node_->get_logger(), "Disarm command call failed: %s", e.what());
      }
    };
    cmd_client_->async_send_request(req, cb);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "cmd_client_ not ready — cannot send disarm!");
  }

  // Backup channel via CommandBool
  if (arm_client_->service_is_ready()) {
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = false;
    arm_client_->async_send_request(req,
      [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture f) {
        try {
          auto res = f.get();
          if (!res->success) {
            RCLCPP_WARN(node_->get_logger(), "CommandBool disarm also rejected");
          }
        } catch (...) {}
      });
  }

  disarm_attempt_time = node_->get_clock()->now().nanoseconds() * 1e-9;
}

void VehicleInterface::set_px4_param_float(const std::string & param_id, float value)
{
  if (!param_set_client_->service_is_ready()) return;
  auto req = std::make_shared<mavros_msgs::srv::ParamSet::Request>();
  req->param_id = param_id;
  req->value.real = static_cast<double>(value);
  req->value.integer = 0;
  param_set_client_->async_send_request(req,
    [this, param_id, value](rclcpp::Client<mavros_msgs::srv::ParamSet>::SharedFuture f) {
      try {
        auto res = f.get();
        if (res->success) {
          RCLCPP_INFO(node_->get_logger(), "Param %s set to %.2f", param_id.c_str(), value);
        } else {
          RCLCPP_WARN(node_->get_logger(), "Param %s set failed", param_id.c_str());
        }
      } catch (...) {}
    });
}

void VehicleInterface::query_px4_params(int & land_mode)
{
  if (param_get_client_->service_is_ready()) {
    auto req = std::make_shared<mavros_msgs::srv::ParamGet::Request>();
    req->param_id = "RTL_PLD_MD";
    auto cb = [this, &land_mode](rclcpp::Client<mavros_msgs::srv::ParamGet>::SharedFuture future_result) {
      try {
        auto res = future_result.get();
        if (res->success) {
          int val = static_cast<int>(res->value.integer);
          if (val == 0 || val == 1 || val == 2) {
            if (land_mode != val) {
              RCLCPP_INFO(node_->get_logger(), "Automatically synced PX4 RTL_PLD_MD param: %d -> %d", land_mode, val);
              land_mode = val;
            }
          }
        }
      } catch (const std::exception & exc) {
        RCLCPP_WARN(node_->get_logger(), "Failed to query PX4 parameter RTL_PLD_MD: %s", exc.what());
      }
    };
    param_get_client_->async_send_request(req, cb);
  }
}

void VehicleInterface::publish_setpoint(const Vector3 & sp_enu, double sp_yaw)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = node_->get_clock()->now();
  msg.header.frame_id = "map";
  msg.pose.position.x = sp_enu.x;
  msg.pose.position.y = sp_enu.y;
  msg.pose.position.z = sp_enu.z;
  msg.pose.orientation.z = std::sin(sp_yaw / 2.0);
  msg.pose.orientation.w = std::cos(sp_yaw / 2.0);
  pub_sp_->publish(msg);
}

}  // namespace precision_landing
