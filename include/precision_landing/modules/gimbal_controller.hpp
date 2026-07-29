#ifndef PRECISION_LANDING__MODULES__GIMBAL_CONTROLLER_HPP_
#define PRECISION_LANDING__MODULES__GIMBAL_CONTROLLER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include "precision_landing/types.hpp"

namespace precision_landing
{

class GimbalController
{
public:
  using CommandLongClient = rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr;

  GimbalController(rclcpp::Node * node, CommandLongClient cmd_client);
  ~GimbalController() = default;

  void tick(PrecLandState current_state);

private:
  void send_command(uint16_t command, float p1 = 0.0f, float p2 = 0.0f, float p3 = 0.0f,
                    float p4 = 0.0f, float p5 = 0.0f, float p6 = 0.0f, float p7 = 0.0f);

  rclcpp::Node * node_;
  CommandLongClient cmd_client_;
  bool gimbal_configured_{false};
};

}  // namespace precision_landing

#endif  // PRECISION_LANDING__MODULES__GIMBAL_CONTROLLER_HPP_
