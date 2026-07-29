#ifndef PRECISION_LANDING__MODULES__PRECISION_LAND_FSM_HPP_
#define PRECISION_LANDING__MODULES__PRECISION_LAND_FSM_HPP_

#include <memory>
#include <string>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "precision_landing/types.hpp"

namespace precision_landing
{

class PrecisionLandFSM
{
public:
  PrecisionLandFSM(rclcpp::Node * node);
  ~PrecisionLandFSM() = default;

  PrecLandState get_state() const { return state_; }
  static std::string to_string(PrecLandState s);

  bool can_transition(PrecLandState from, PrecLandState to) const;
  bool transition(PrecLandState new_state);
  void publish_state();

  // State Timers & History Helpers
  std::optional<double> search_start;
  std::optional<double> align_start;
  std::optional<double> target_lost_start;
  PrecLandState target_lost_from{PrecLandState::HORIZONTAL_APPROACH};
  double final_approach_start{0.0};
  double yaw_lock_stage_start{0.0};

private:
  rclcpp::Node * node_;
  PrecLandState state_{PrecLandState::IDLE};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
};

}  // namespace precision_landing

#endif  // PRECISION_LANDING__MODULES__PRECISION_LAND_FSM_HPP_
