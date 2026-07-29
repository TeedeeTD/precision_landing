#include "precision_landing/modules/guidance_controller.hpp"
#include <cmath>
#include <algorithm>

namespace precision_landing
{

void GuidanceController::reset(const Vector3 & pos_enu, double initial_yaw)
{
  sp_prev_ = pos_enu;
  sp_prev_vel_ = Vector3{0.0, 0.0, 0.0};
  sp_yaw_ = initial_yaw;
}

double GuidanceController::get_yaw(const Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

Quaternion GuidanceController::quaternion_multiply(const Quaternion & q1, const Quaternion & q2)
{
  Quaternion r;
  r.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
  r.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
  r.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
  r.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
  return r;
}

double GuidanceController::wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double GuidanceController::circular_mean(const std::vector<double> & angles)
{
  double s = 0.0;
  double c = 0.0;
  for (double a : angles) {
    s += std::sin(a);
    c += std::cos(a);
  }
  return std::atan2(s, c);
}

Vector3 GuidanceController::calculate_visual_setpoint(
  const Vector3 & pos_enu,
  const std::optional<std::tuple<double, double>> & target_val,
  double z_sp, double max_step, double servo_gain)
{
  if (!target_val.has_value()) {
    return Vector3{pos_enu.x, pos_enu.y, z_sp};
  }
  double rel_x = std::get<0>(target_val.value()) - pos_enu.x;
  double rel_y = std::get<1>(target_val.value()) - pos_enu.y;
  double delta_x = servo_gain * rel_x;
  double delta_y = servo_gain * rel_y;
  double d = std::sqrt(delta_x*delta_x + delta_y*delta_y);
  if (d > max_step) {
    delta_x *= max_step / d;
    delta_y *= max_step / d;
  }
  return Vector3{
    pos_enu.x + delta_x,
    pos_enu.y + delta_y,
    z_sp
  };
}

Vector3 GuidanceController::apply_slew_rate(const Vector3 & target_sp, double dt, double sp_vel_max, double sp_accel_max)
{
  if (dt <= 0.0) return target_sp;

  double vx_des = (target_sp.x - sp_prev_.x) / dt;
  double vy_des = (target_sp.y - sp_prev_.y) / dt;

  double v_des_norm = std::sqrt(vx_des * vx_des + vy_des * vy_des);
  if (v_des_norm > sp_vel_max) {
    vx_des = (vx_des / v_des_norm) * sp_vel_max;
    vy_des = (vy_des / v_des_norm) * sp_vel_max;
  }

  double ax = (vx_des - sp_prev_vel_.x) / dt;
  double ay = (vy_des - sp_prev_vel_.y) / dt;

  double a_norm = std::sqrt(ax * ax + ay * ay);
  if (a_norm > sp_accel_max) {
    ax = (ax / a_norm) * sp_accel_max;
    ay = (ay / a_norm) * sp_accel_max;
  }

  sp_prev_vel_.x += ax * dt;
  sp_prev_vel_.y += ay * dt;

  Vector3 filtered_sp;
  filtered_sp.x = sp_prev_.x + sp_prev_vel_.x * dt;
  filtered_sp.y = sp_prev_.y + sp_prev_vel_.y * dt;
  filtered_sp.z = target_sp.z;

  sp_prev_ = filtered_sp;
  return filtered_sp;
}

double GuidanceController::current_descent_rate(double alt, const GuidanceParams & params)
{
  if (alt > params.mpc_land_alt1) {
    return params.mpc_z_vel_max_dn;
  } else if (alt > params.mpc_land_alt2) {
    return params.mpc_land_speed;
  } else if (alt > params.mpc_land_alt_crawl) {
    double span = params.mpc_land_alt2 - params.mpc_land_alt_crawl;
    double t = (alt - params.mpc_land_alt_crawl) / span;
    return params.mpc_land_crwl + (params.mpc_land_speed - params.mpc_land_crwl) * t;
  } else {
    return params.mpc_land_crwl;
  }
}

double GuidanceController::compute_locked_yaw(const std::vector<double> & yaw_buf, double current_sp_yaw,
                                               double tag_yaw_sign, double tag_yaw_offset)
{
  if (yaw_buf.empty()) {
    return current_sp_yaw;
  }
  double avg_yaw = circular_mean(yaw_buf);
  if (tag_yaw_sign < 0.0) {
    avg_yaw = wrap_angle(avg_yaw + M_PI);
  }
  return wrap_angle(avg_yaw + tag_yaw_offset);
}

void GuidanceController::update_yaw(bool align_yaw_to_tag, const std::optional<double> & tag_yaw_abs,
                                    double held_yaw, double yaw_slew_rate, int ctrl_hz)
{
  double desired = held_yaw;
  if (align_yaw_to_tag && tag_yaw_abs.has_value()) {
    desired = tag_yaw_abs.value();
  }
  double step = yaw_slew_rate / ctrl_hz;
  double err = wrap_angle(desired - sp_yaw_);
  if (std::abs(err) <= step) {
    sp_yaw_ = desired;
  } else {
    sp_yaw_ = wrap_angle(sp_yaw_ + (err > 0 ? step : -step));
  }
}

}  // namespace precision_landing
