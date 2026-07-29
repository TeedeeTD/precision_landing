#ifndef PRECISION_LANDING__MODULES__GUIDANCE_CONTROLLER_HPP_
#define PRECISION_LANDING__MODULES__GUIDANCE_CONTROLLER_HPP_

#include <vector>
#include <tuple>
#include <optional>
#include "precision_landing/types.hpp"

namespace precision_landing
{

struct GuidanceParams
{
  double sp_vel_max{1.0};
  double sp_accel_max{0.5};
  double yaw_slew_rate{0.2};
  double tag_yaw_sign{1.0};
  double tag_yaw_offset{0.0};
  double mpc_land_alt1{10.0};
  double mpc_land_alt2{5.0};
  double mpc_land_alt_crawl{1.0};
  double mpc_z_vel_max_dn{1.5};
  double mpc_land_speed{0.7};
  double mpc_land_crwl{0.3};
};

class GuidanceController
{
public:
  GuidanceController() = default;
  ~GuidanceController() = default;

  void reset(const Vector3 & pos_enu, double initial_yaw);

  // Math & Angle Helpers
  static double get_yaw(const Quaternion & q);
  static Quaternion quaternion_multiply(const Quaternion & q1, const Quaternion & q2);
  static double wrap_angle(double angle);
  static double circular_mean(const std::vector<double> & angles);

  // Controller Functions
  Vector3 calculate_visual_setpoint(const Vector3 & pos_enu,
                                     const std::optional<std::tuple<double, double>> & target_val,
                                     double z_sp, double max_step, double servo_gain);

  Vector3 apply_slew_rate(const Vector3 & target_sp, double dt, double sp_vel_max, double sp_accel_max);

  double current_descent_rate(double alt, const GuidanceParams & params);

  double compute_locked_yaw(const std::vector<double> & yaw_buf, double current_sp_yaw,
                            double tag_yaw_sign, double tag_yaw_offset);

  void update_yaw(bool align_yaw_to_tag, const std::optional<double> & tag_yaw_abs,
                  double held_yaw, double yaw_slew_rate, int ctrl_hz);

  // Getters & Setters
  double get_sp_yaw() const { return sp_yaw_; }
  void set_sp_yaw(double yaw) { sp_yaw_ = yaw; }
  Vector3 get_sp_prev() const { return sp_prev_; }
  void set_sp_prev(const Vector3 & sp) { sp_prev_ = sp; }
  Vector3 get_sp_prev_vel() const { return sp_prev_vel_; }
  void set_sp_prev_vel(const Vector3 & vel) { sp_prev_vel_ = vel; }

private:
  Vector3 sp_prev_{0.0, 0.0, 0.0};
  Vector3 sp_prev_vel_{0.0, 0.0, 0.0};
  double sp_yaw_{0.0};
};

}  // namespace precision_landing

#endif  // PRECISION_LANDING__MODULES__GUIDANCE_CONTROLLER_HPP_
