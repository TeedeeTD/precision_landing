#ifndef PRECISION_LANDING__MODULES__VEHICLE_INTERFACE_HPP_
#define PRECISION_LANDING__MODULES__VEHICLE_INTERFACE_HPP_

#include <memory>
#include <deque>
#include <tuple>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/param_get.hpp>
#include <mavros_msgs/srv/param_set.hpp>
#include <mavros_msgs/srv/waypoint_pull.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "precision_landing/types.hpp"

namespace precision_landing
{

class VehicleInterface : public std::enable_shared_from_this<VehicleInterface>
{
public:
  VehicleInterface(rclcpp::Node * node);
  ~VehicleInterface() = default;

  void init_ros_interfaces(
    rclcpp::QoS pose_qos, rclcpp::QoS state_qos,
    std::function<void(const geometry_msgs::msg::PoseStamped::SharedPtr)> pos_cb,
    std::function<void(const mavros_msgs::msg::State::SharedPtr)> state_cb,
    std::function<void(const mavros_msgs::msg::ExtendedState::SharedPtr)> ext_state_cb,
    std::function<void(const mavros_msgs::msg::WaypointList::SharedPtr)> wp_cb
  );

  // MAVROS / PX4 Service Helpers
  void set_mode(const std::string & mode, bool & offboard_activated);
  void send_command(uint16_t command, float p1 = 0.0f, float p2 = 0.0f, float p3 = 0.0f,
                    float p4 = 0.0f, float p5 = 0.0f, float p6 = 0.0f, float p7 = 0.0f);
  void disarm(double & disarm_attempt_time);
  void set_px4_param_float(const std::string & param_id, float value);
  void query_px4_params(int & land_mode);
  void pull_waypoints_immediately();

  void publish_setpoint(const Vector3 & sp_enu, double sp_yaw);

  // Getters & Accessors
  Vector3 get_pos_enu() const { return pos_enu_; }
  Quaternion get_q_att() const { return q_att_; }
  std::string get_current_mode() const { return current_mode_; }
  uint8_t get_landed_state() const { return landed_state_; }
  bool is_landing() const { return is_landing_; }
  bool is_armed() const { return armed_; }
  bool is_mavros_connected() const { return mavros_connected_; }
  const std::deque<std::tuple<double, Vector3, Quaternion>> & get_history() const { return history_; }
  const std::vector<mavros_msgs::msg::Waypoint> & get_waypoints() const { return waypoints_; }
  uint16_t get_current_wp_seq() const { return current_wp_seq_; }
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr get_cmd_client() const { return cmd_client_; }

  void set_pos_enu(const Vector3 & pos) { pos_enu_ = pos; }
  void set_q_att(const Quaternion & q) { q_att_ = q; }
  void set_current_mode(const std::string & m) { current_mode_ = m; }
  void set_landed_state(uint8_t ls) { landed_state_ = ls; }
  void set_is_landing(bool l) { is_landing_ = l; }
  void set_armed(bool a) { armed_ = a; }
  void set_mavros_connected(bool c) { mavros_connected_ = c; }
  void push_history(double stamp, const Vector3 & pos, const Quaternion & q);

private:
  rclcpp::Node * node_;

  Vector3 pos_enu_{0.0, 0.0, 0.0};
  Quaternion q_att_{1.0, 0.0, 0.0, 0.0};
  std::string current_mode_;
  uint8_t landed_state_{0};
  bool is_landing_{false};
  bool armed_{false};
  bool mavros_connected_{false};

  std::deque<std::tuple<double, Vector3, Quaternion>> history_;
  std::vector<mavros_msgs::msg::Waypoint> waypoints_;
  uint16_t current_wp_seq_{0};

  // ROS 2 Interfaces
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_sp_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr pub_sp_raw_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pos_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr sub_state_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr sub_ext_state_;
  rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr sub_waypoints_;

  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr cmd_client_;
  rclcpp::Client<mavros_msgs::srv::ParamGet>::SharedPtr param_get_client_;
  rclcpp::Client<mavros_msgs::srv::ParamSet>::SharedPtr param_set_client_;
  rclcpp::Client<mavros_msgs::srv::WaypointPull>::SharedPtr wp_pull_client_;
};

}  // namespace precision_landing

#endif  // PRECISION_LANDING__MODULES__VEHICLE_INTERFACE_HPP_
