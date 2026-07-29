#include "precision_landing/modules/precision_land_fsm.hpp"

namespace precision_landing
{

PrecisionLandFSM::PrecisionLandFSM(rclcpp::Node * node)
: node_(node)
{
  pub_state_ = node_->create_publisher<std_msgs::msg::String>("/lander/state", 10);
}

std::string PrecisionLandFSM::to_string(PrecLandState s)
{
  switch (s) {
    case PrecLandState::IDLE: return "IDLE";
    case PrecLandState::START: return "START";
    case PrecLandState::HORIZONTAL_APPROACH: return "HORIZONTAL_APPROACH";
    case PrecLandState::DESCEND_ABOVE_TARGET: return "DESCEND_ABOVE_TARGET";
    case PrecLandState::FINAL_APPROACH: return "FINAL_APPROACH";
    case PrecLandState::SEARCH: return "SEARCH";
    case PrecLandState::TARGET_LOST: return "TARGET_LOST";
    case PrecLandState::FALLBACK: return "FALLBACK";
    case PrecLandState::DONE: return "DONE";
  }
  return "UNKNOWN";
}

bool PrecisionLandFSM::can_transition(PrecLandState from, PrecLandState to) const
{
  if (to == PrecLandState::IDLE) return true;
  if (to == PrecLandState::DONE) return true;
  if (to == PrecLandState::FALLBACK) return true;

  switch (from) {
    case PrecLandState::IDLE:
      return (to == PrecLandState::START);
    case PrecLandState::START:
      return (to == PrecLandState::HORIZONTAL_APPROACH || to == PrecLandState::SEARCH);
    case PrecLandState::HORIZONTAL_APPROACH:
      return (to == PrecLandState::DESCEND_ABOVE_TARGET || to == PrecLandState::TARGET_LOST);
    case PrecLandState::DESCEND_ABOVE_TARGET:
      return (to == PrecLandState::FINAL_APPROACH || to == PrecLandState::TARGET_LOST ||
              to == PrecLandState::SEARCH || to == PrecLandState::HORIZONTAL_APPROACH);
    case PrecLandState::FINAL_APPROACH:
      return false;
    case PrecLandState::SEARCH:
      return (to == PrecLandState::HORIZONTAL_APPROACH);
    case PrecLandState::TARGET_LOST:
      return (to == PrecLandState::HORIZONTAL_APPROACH || to == PrecLandState::DESCEND_ABOVE_TARGET ||
              to == PrecLandState::SEARCH || to == PrecLandState::FINAL_APPROACH);
    case PrecLandState::FALLBACK:
      return false;
    case PrecLandState::DONE:
      return (to == PrecLandState::IDLE);
  }
  return false;
}

bool PrecisionLandFSM::transition(PrecLandState new_state)
{
  if (!can_transition(state_, new_state)) {
    RCLCPP_WARN(node_->get_logger(), "FSM: transition from %s to %s rejected by guard",
                to_string(state_).c_str(), to_string(new_state).c_str());
    return false;
  }

  PrecLandState old = state_;
  state_ = new_state;

  RCLCPP_INFO(node_->get_logger(), "FSM: %s → %s", to_string(old).c_str(), to_string(new_state).c_str());
  return true;
}

void PrecisionLandFSM::publish_state()
{
  try {
    std_msgs::msg::String m;
    m.data = to_string(state_);
    pub_state_->publish(m);
  } catch (...) {
  }
}

}  // namespace precision_landing
