#include "precision_landing/modules/gimbal_controller.hpp"
#include <limits>

namespace precision_landing
{

GimbalController::GimbalController(rclcpp::Node * node, CommandLongClient cmd_client)
: node_(node), cmd_client_(cmd_client)
{
}

void GimbalController::send_command(uint16_t command, float p1, float p2, float p3, float p4, float p5, float p6, float p7)
{
  if (cmd_client_ && cmd_client_->service_is_ready()) {
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

void GimbalController::tick(PrecLandState current_state)
{
  if (!cmd_client_ || !cmd_client_->service_is_ready()) {
    return;
  }
  if (!gimbal_configured_) {
    send_command(1001, 1.0f, 191.0f); // configure gimbal
    gimbal_configured_ = true;
  }
  float pitch = (current_state != PrecLandState::IDLE && current_state != PrecLandState::DONE) ? -90.0f : 0.0f;
  send_command(1000, pitch, 0.0f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), 0.0f);
  send_command(205, pitch, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f);
}

}  // namespace precision_landing
