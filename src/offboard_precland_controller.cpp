#include "precision_landing/offboard_precland_controller.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

namespace precision_landing
{

OffboardPreclandController::OffboardPreclandController(const rclcpp::NodeOptions & options)
: Node("offboard_precland_controller", options)
{
  // --- Declare Parameters ---
  this->declare_parameter<double>("camera_x_to_body_east_sign");
  this->declare_parameter<double>("camera_y_to_body_north_sign");
  this->declare_parameter<std::string>("camera_yaw_frame");
  this->declare_parameter<double>("camera_offset_x");
  this->declare_parameter<double>("camera_offset_y");
  this->declare_parameter<double>("camera_offset_z");
  this->declare_parameter<double>("marker_size");
  this->declare_parameter<std::string>("target_topic");
  this->declare_parameter<std::string>("target_pose_topic");
  this->declare_parameter<bool>("align_yaw_to_tag");
  this->declare_parameter<double>("tag_yaw_sign");
  this->declare_parameter<double>("tag_yaw_offset");
  this->declare_parameter<int>("land_mode");
  this->declare_parameter<double>("abort_alt");
  this->declare_parameter<double>("final_alt");
  this->declare_parameter<double>("yaw_slew_rate");
  this->declare_parameter<int>("yaw_lock_samples");
  this->declare_parameter<double>("yaw_lock_alt");
  this->declare_parameter<double>("yaw_lock_alt_2");

  this->declare_parameter<int>("ctrl_hz");
  this->declare_parameter<double>("target_timeout");
  this->declare_parameter<int>("tracking_confirm");
  this->declare_parameter<int>("align_confirm");
  this->declare_parameter<double>("align_timeout");
  this->declare_parameter<double>("search_timeout");
  this->declare_parameter<int>("max_search");
  this->declare_parameter<double>("descent_rate");
  this->declare_parameter<double>("fappr_alt");
  this->declare_parameter<double>("hacc_rad");
  this->declare_parameter<double>("max_align_step");
  this->declare_parameter<double>("max_descent_step");
  this->declare_parameter<double>("servo_gain_high");
  this->declare_parameter<double>("servo_gain_low");

  this->declare_parameter<double>("mpc_land_alt1");
  this->declare_parameter<double>("mpc_land_alt2");
  this->declare_parameter<double>("mpc_land_alt_crawl");
  this->declare_parameter<double>("mpc_z_vel_max_dn");
  this->declare_parameter<double>("mpc_land_speed");
  this->declare_parameter<double>("mpc_land_crwl");
  this->declare_parameter<double>("high_alt");
  this->declare_parameter<double>("low_alt");
  this->declare_parameter<double>("alpha_high");
  this->declare_parameter<double>("alpha_low");
  this->declare_parameter<int>("filter_window");

  this->declare_parameter<double>("align_r_min");
  this->declare_parameter<double>("align_r_max");
  this->declare_parameter<double>("align_r_gain");
  this->declare_parameter<double>("align_r_bias");
  this->declare_parameter<double>("desc_r_min");
  this->declare_parameter<double>("desc_r_max");
  this->declare_parameter<double>("desc_r_gain");
  this->declare_parameter<double>("desc_r_bias");
  this->declare_parameter<double>("reject_r_min");
  this->declare_parameter<double>("reject_r_max");
  this->declare_parameter<double>("reject_r_gain");

  this->declare_parameter<double>("target_loss_grace");
  this->declare_parameter<double>("descent_loss_hold");
  this->declare_parameter<double>("low_alt_max_err");
  this->declare_parameter<double>("search_alt");
  this->declare_parameter<double>("search_alt_max");
  this->declare_parameter<double>("final_approach_timeout");
  this->declare_parameter<double>("final_descent_rate");
  this->declare_parameter<double>("final_align_step");
  this->declare_parameter<double>("sp_vel_max");
  this->declare_parameter<double>("sp_accel_max");
  this->declare_parameter<double>("yaw_lock_timeout");
  this->declare_parameter<int>("yaw_lock_min_samples");
  this->declare_parameter<double>("camera_mount_roll");
  this->declare_parameter<double>("camera_mount_pitch");
  this->declare_parameter<double>("camera_mount_yaw");

  // --- Get Parameters ---
  cam_params_.camera_x_to_body_east_sign = this->get_parameter("camera_x_to_body_east_sign").as_double();
  cam_params_.camera_y_to_body_north_sign = this->get_parameter("camera_y_to_body_north_sign").as_double();
  cam_params_.camera_yaw_frame = this->get_parameter("camera_yaw_frame").as_string();
  cam_params_.camera_offset_x = this->get_parameter("camera_offset_x").as_double();
  cam_params_.camera_offset_y = this->get_parameter("camera_offset_y").as_double();
  cam_params_.camera_offset_z = this->get_parameter("camera_offset_z").as_double();
  cam_params_.camera_mount_roll = this->get_parameter("camera_mount_roll").as_double();
  cam_params_.camera_mount_pitch = this->get_parameter("camera_mount_pitch").as_double();
  cam_params_.camera_mount_yaw = this->get_parameter("camera_mount_yaw").as_double();

  target_topic_ = this->get_parameter("target_topic").as_string();
  target_pose_topic_ = this->get_parameter("target_pose_topic").as_string();
  align_yaw_to_tag_ = this->get_parameter("align_yaw_to_tag").as_bool();
  guidance_params_.tag_yaw_sign = this->get_parameter("tag_yaw_sign").as_double();
  guidance_params_.tag_yaw_offset = this->get_parameter("tag_yaw_offset").as_double();
  land_mode_ = this->get_parameter("land_mode").as_int();
  abort_alt_param_ = this->get_parameter("abort_alt").as_double();
  final_alt_param_ = this->get_parameter("final_alt").as_double();
  guidance_params_.yaw_slew_rate = this->get_parameter("yaw_slew_rate").as_double();
  yaw_lock_samples_ = this->get_parameter("yaw_lock_samples").as_int();
  yaw_lock_alt_ = this->get_parameter("yaw_lock_alt").as_double();
  yaw_lock_alt_2_ = this->get_parameter("yaw_lock_alt_2").as_double();

  ctrl_hz_ = this->get_parameter("ctrl_hz").as_int();
  target_timeout_ = this->get_parameter("target_timeout").as_double();
  tracking_confirm_ = this->get_parameter("tracking_confirm").as_int();
  align_confirm_ = this->get_parameter("align_confirm").as_int();
  align_timeout_ = this->get_parameter("align_timeout").as_double();
  search_timeout_ = this->get_parameter("search_timeout").as_double();
  max_search_ = this->get_parameter("max_search").as_int();
  descent_rate_ = this->get_parameter("descent_rate").as_double();
  fappr_alt_ = this->get_parameter("fappr_alt").as_double();
  hacc_rad_ = this->get_parameter("hacc_rad").as_double();
  max_align_step_ = this->get_parameter("max_align_step").as_double();
  max_descent_step_ = this->get_parameter("max_descent_step").as_double();

  gate_params_.servo_gain_high = this->get_parameter("servo_gain_high").as_double();
  gate_params_.servo_gain_low = this->get_parameter("servo_gain_low").as_double();
  guidance_params_.mpc_land_alt1 = this->get_parameter("mpc_land_alt1").as_double();
  guidance_params_.mpc_land_alt2 = this->get_parameter("mpc_land_alt2").as_double();
  guidance_params_.mpc_land_alt_crawl = this->get_parameter("mpc_land_alt_crawl").as_double();
  guidance_params_.mpc_z_vel_max_dn = this->get_parameter("mpc_z_vel_max_dn").as_double();
  guidance_params_.mpc_land_speed = this->get_parameter("mpc_land_speed").as_double();
  guidance_params_.mpc_land_crwl = this->get_parameter("mpc_land_crwl").as_double();

  gate_params_.high_alt = this->get_parameter("high_alt").as_double();
  gate_params_.low_alt = this->get_parameter("low_alt").as_double();
  gate_params_.alpha_high = this->get_parameter("alpha_high").as_double();
  gate_params_.alpha_low = this->get_parameter("alpha_low").as_double();

  gate_params_.align_r_min = this->get_parameter("align_r_min").as_double();
  gate_params_.align_r_max = this->get_parameter("align_r_max").as_double();
  gate_params_.align_r_gain = this->get_parameter("align_r_gain").as_double();
  gate_params_.align_r_bias = this->get_parameter("align_r_bias").as_double();
  gate_params_.desc_r_min = this->get_parameter("desc_r_min").as_double();
  gate_params_.desc_r_max = this->get_parameter("desc_r_max").as_double();
  gate_params_.desc_r_gain = this->get_parameter("desc_r_gain").as_double();
  gate_params_.desc_r_bias = this->get_parameter("desc_r_bias").as_double();
  gate_params_.reject_r_min = this->get_parameter("reject_r_min").as_double();
  gate_params_.reject_r_max = this->get_parameter("reject_r_max").as_double();
  gate_params_.reject_r_gain = this->get_parameter("reject_r_gain").as_double();

  target_loss_grace_ = this->get_parameter("target_loss_grace").as_double();
  descent_loss_hold_ = this->get_parameter("descent_loss_hold").as_double();
  low_alt_max_err_ = this->get_parameter("low_alt_max_err").as_double();
  search_alt_ = this->get_parameter("search_alt").as_double();
  search_alt_max_ = this->get_parameter("search_alt_max").as_double();
  final_approach_timeout_ = this->get_parameter("final_approach_timeout").as_double();
  final_descent_rate_ = this->get_parameter("final_descent_rate").as_double();
  final_align_step_ = this->get_parameter("final_align_step").as_double();
  guidance_params_.sp_vel_max = this->get_parameter("sp_vel_max").as_double();
  guidance_params_.sp_accel_max = this->get_parameter("sp_accel_max").as_double();
  yaw_lock_timeout_ = this->get_parameter("yaw_lock_timeout").as_double();
  yaw_lock_min_samples_ = this->get_parameter("yaw_lock_min_samples").as_int();

  // --- Initialize 5 Sub-Modules ---
  vehicle_ = std::make_shared<VehicleInterface>(this);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  target_tracker_ = std::make_unique<TargetTracker>(this, tf_buffer_.get());

  fsm_ = std::make_unique<PrecisionLandFSM>(this);
  guidance_ = std::make_unique<GuidanceController>();
  gimbal_ = std::make_unique<GimbalController>(this, vehicle_->get_cmd_client());

  // --- QoS Profiles ---
  rmw_qos_profile_t pose_qos_profile = rmw_qos_profile_default;
  pose_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  pose_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  pose_qos_profile.depth = 1;
  auto pose_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(pose_qos_profile), pose_qos_profile);

  rmw_qos_profile_t state_qos_profile = rmw_qos_profile_default;
  state_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  state_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  state_qos_profile.depth = 1;
  auto state_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(state_qos_profile), state_qos_profile);

  // Initialize Vehicle Subscriptions & Services
  vehicle_->init_ros_interfaces(
    pose_qos, state_qos,
    std::bind(&OffboardPreclandController::on_pos, this, std::placeholders::_1),
    std::bind(&OffboardPreclandController::on_state, this, std::placeholders::_1),
    std::bind(&OffboardPreclandController::on_ext_state, this, std::placeholders::_1),
    std::bind(&OffboardPreclandController::on_waypoints, this, std::placeholders::_1)
  );

  sub_target_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    target_pose_topic_, 10,
    std::bind(&OffboardPreclandController::on_target, this, std::placeholders::_1)
  );

  // --- Timers ---
  double timer_period_sec = 1.0 / ctrl_hz_;
  loop_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period_sec),
    std::bind(&OffboardPreclandController::control_loop, this)
  );

  gimbal_timer_ = this->create_wall_timer(
    std::chrono::seconds(2),
    std::bind(&OffboardPreclandController::gimbal_tick, this)
  );

  param_timer_ = this->create_wall_timer(
    std::chrono::seconds(3),
    std::bind(&OffboardPreclandController::query_px4_params, this)
  );

  RCLCPP_INFO(this->get_logger(), "OffboardPreclandController (Modular) C++ ready — monitoring for AUTO.LAND");
}

double OffboardPreclandController::now_sec()
{
  return this->get_clock()->now().nanoseconds() * 1e-9;
}

void OffboardPreclandController::transition(PrecLandState new_state)
{
  if (!fsm_->transition(new_state)) {
    return;
  }

  if (new_state == PrecLandState::IDLE || new_state == PrecLandState::START) {
    yaw_locked_ = false;
    tag_yaw_abs_.reset();
    yaw_lock_buf_.clear();
    yaw_realign_complete_ = false;
    realign_cnt_ = 0;
    yaw_lock_stage_ = 0;
    target_tracker_->set_target_enu_filtered(std::nullopt);
  }

  if (new_state == PrecLandState::START) {
    guidance_->reset(vehicle_->get_pos_enu(), held_yaw_);
    disarm_requested_ = false;
    auto_land_fallback_sent_ = false;
    target_tracker_->reset_virtual_pad_z();
  }

  if (new_state == PrecLandState::FINAL_APPROACH) {
    auto target_val = target_tracker_->get_target_enu_filtered().has_value() ?
                      target_tracker_->get_target_enu_filtered() : target_tracker_->get_target_enu();
    if (target_val.has_value()) {
      final_x_ = std::get<0>(target_val.value());
      final_y_ = std::get<1>(target_val.value());
    } else {
      final_x_ = vehicle_->get_pos_enu().x;
      final_y_ = vehicle_->get_pos_enu().y;
    }
    fsm_->final_approach_start = now_sec();
    final_approach_entry_z_ = vehicle_->get_pos_enu().z;
    guidance_->set_sp_prev(vehicle_->get_pos_enu());
    guidance_->set_sp_prev_vel(Vector3{0.0, 0.0, 0.0});
  }
}

// ── Callbacks ──────────────────────────────────────────────

void OffboardPreclandController::on_pos(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  Vector3 pos;
  pos.x = msg->pose.position.x;
  pos.y = msg->pose.position.y;
  pos.z = msg->pose.position.z;

  Quaternion q;
  q.w = msg->pose.orientation.w;
  q.x = msg->pose.orientation.x;
  q.y = msg->pose.orientation.y;
  q.z = msg->pose.orientation.z;

  vehicle_->set_pos_enu(pos);
  vehicle_->set_q_att(q);

  double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  vehicle_->push_history(stamp, pos, q);
}

void OffboardPreclandController::on_state(const mavros_msgs::msg::State::SharedPtr msg)
{
  vehicle_->set_current_mode(msg->mode);
  vehicle_->set_armed(msg->armed);
  bool was_connected = vehicle_->is_mavros_connected();
  vehicle_->set_mavros_connected(msg->connected);

  if (msg->connected && !was_connected) {
    RCLCPP_INFO(this->get_logger(), "MAVROS connected — pulling waypoints...");
    vehicle_->pull_waypoints_immediately();
  }

  bool was_landing = vehicle_->is_landing();
  bool is_landing = (msg->mode == "AUTO.LAND");
  vehicle_->set_is_landing(is_landing);

  if (is_landing != was_landing) {
    RCLCPP_INFO(this->get_logger(), "Landing flag: %s (mode=%s)", is_landing ? "true" : "false", msg->mode.c_str());
    if (is_landing) {
      vehicle_->pull_waypoints_immediately();
    }
  }
}

void OffboardPreclandController::on_ext_state(const mavros_msgs::msg::ExtendedState::SharedPtr msg)
{
  vehicle_->set_landed_state(msg->landed_state);
  bool was_landing = vehicle_->is_landing();
  if (!was_landing && msg->landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_LANDING) {
    vehicle_->set_is_landing(true);
  }

  if (vehicle_->is_landing() != was_landing) {
    RCLCPP_INFO(this->get_logger(), "Landing flag: %s (landed_state=%d)", vehicle_->is_landing() ? "true" : "false", msg->landed_state);
    if (vehicle_->is_landing()) {
      vehicle_->pull_waypoints_immediately();
    }
  }
}

void OffboardPreclandController::on_waypoints(const mavros_msgs::msg::WaypointList::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received waypoints update: %d items. Active seq: %d", (int)msg->waypoints.size(), msg->current_seq);
}

void OffboardPreclandController::on_target(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  target_tracker_->publish_static_transform(msg->header.frame_id, cam_params_);
  target_tracker_->process_target_msg(
    msg, vehicle_->get_pos_enu(), vehicle_->get_q_att(),
    vehicle_->get_history(), align_yaw_to_tag_,
    fsm_->get_state(), yaw_lock_samples_, yaw_lock_alt_, yaw_lock_alt_2_,
    yaw_lock_stage_, yaw_locked_, yaw_lock_buf_, tag_yaw_abs_, guidance_->get_sp_yaw()
  );
}

void OffboardPreclandController::gimbal_tick()
{
  gimbal_->tick(fsm_->get_state());
}

void OffboardPreclandController::query_px4_params()
{
  vehicle_->query_px4_params(land_mode_);
}

// ── Control Loop FSM ──────────────────────────────────────

void OffboardPreclandController::control_loop()
{
  double now = now_sec();
  double wall_now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  if (last_loop_run_time_ > 0.0) {
    double dt_loop = wall_now - last_loop_run_time_;
    if (dt_loop > 0.2) {
      RCLCPP_WARN(this->get_logger(), "Watchdog: Control loop delayed abnormally by %.3fs (wall time)!", dt_loop);
      if (fsm_->get_state() != PrecLandState::IDLE && fsm_->get_state() != PrecLandState::DONE && fsm_->get_state() != PrecLandState::FALLBACK) {
        RCLCPP_ERROR(this->get_logger(), "Watchdog triggered: transitioning to FALLBACK");
        transition(PrecLandState::FALLBACK);
        last_loop_run_time_ = wall_now;
        return;
      }
    }
  }
  last_loop_run_time_ = wall_now;

  PrecLandState st = fsm_->get_state();

  if (st == PrecLandState::FINAL_APPROACH) {
    if (!vehicle_->is_armed()) {
      RCLCPP_INFO(this->get_logger(), "Drone disarmed. Landing complete.");
      disarm_requested_ = false;
      transition(PrecLandState::DONE);
      return;
    }

    if (disarm_requested_ && vehicle_->is_armed()) {
      if ((now - disarm_attempt_time_) >= 0.2) {
        RCLCPP_WARN(this->get_logger(), "Retrying force-disarm (%.1fs since first attempt)",
                    now - disarm_attempt_time_first_);
        vehicle_->disarm(disarm_attempt_time_);
      }

      double since_first = now - disarm_attempt_time_first_;
      if (since_first > 2.0 && !auto_land_fallback_sent_) {
        RCLCPP_ERROR(this->get_logger(),
          "Force-disarm not confirmed after 2s — escalating to AUTO.LAND as last resort");
        vehicle_->set_mode("AUTO.LAND", offboard_activated_);
        auto_land_fallback_sent_ = true;
      }
    }

    if (!disarm_requested_ &&
        vehicle_->get_landed_state() == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND) {
      RCLCPP_INFO(this->get_logger(), "landed_state=ON_GROUND detected → force-disarm");
      disarm_requested_ = true;
      disarm_attempt_time_first_ = now_sec();
      vehicle_->disarm(disarm_attempt_time_);
    }
  }

  if (!target_tracker_->is_target_fresh(now, target_timeout_)) {
    target_tracker_->reset_tracking_count();
  }

  switch (st) {
    case PrecLandState::IDLE:                 st_idle(); break;
    case PrecLandState::START:                st_start(); break;
    case PrecLandState::HORIZONTAL_APPROACH:   st_horizontal_approach(); break;
    case PrecLandState::DESCEND_ABOVE_TARGET:  st_descend_above_target(); break;
    case PrecLandState::FINAL_APPROACH:        st_final_approach(); break;
    case PrecLandState::SEARCH:               st_search(); break;
    case PrecLandState::TARGET_LOST:          st_target_lost(); break;
    case PrecLandState::FALLBACK:             st_fallback(); break;
    case PrecLandState::DONE:                 st_done(); break;
  }

  // Target slew rate filtering
  auto target_enu = target_tracker_->get_target_enu();
  if (target_enu.has_value() && target_tracker_->is_target_fresh(now, target_timeout_)) {
    double target_x = std::get<0>(target_enu.value());
    double target_y = std::get<1>(target_enu.value());
    double dt = 1.0 / ctrl_hz_;

    if (!target_tracker_->get_target_enu_filtered().has_value()) {
      guidance_->set_sp_prev(Vector3{target_x, target_y, 0.0});
      guidance_->set_sp_prev_vel(Vector3{0.0, 0.0, 0.0});
      target_tracker_->set_target_enu_filtered(std::make_tuple(target_x, target_y));
    } else {
      Vector3 filt = guidance_->apply_slew_rate(Vector3{target_x, target_y, 0.0}, dt, guidance_params_.sp_vel_max, guidance_params_.sp_accel_max);
      target_tracker_->set_target_enu_filtered(std::make_tuple(filt.x, filt.y));
    }
  } else {
    target_tracker_->set_target_enu_filtered(std::nullopt);
    guidance_->set_sp_prev(vehicle_->get_pos_enu());
    guidance_->set_sp_prev_vel(Vector3{0.0, 0.0, 0.0});
  }

  guidance_->update_yaw(align_yaw_to_tag_, tag_yaw_abs_, held_yaw_, guidance_params_.yaw_slew_rate, ctrl_hz_);

  st = fsm_->get_state();
  if (st != PrecLandState::IDLE && st != PrecLandState::DONE && st != PrecLandState::FALLBACK) {
    if (!(st == PrecLandState::FINAL_APPROACH && disarm_requested_)) {
      vehicle_->publish_setpoint(sp_enu_, guidance_->get_sp_yaw());
    }
  }

  fsm_->publish_state();
}

// ── State Handlers ────────────────────────────────────────

void OffboardPreclandController::st_idle()
{
  if (!(vehicle_->is_landing() && vehicle_->is_armed())) {
    return;
  }

  int active_mode = land_mode_;
  const auto & waypoints = vehicle_->get_waypoints();
  uint16_t current_wp_seq = vehicle_->get_current_wp_seq();

  for (size_t idx : {static_cast<size_t>(current_wp_seq), static_cast<size_t>(current_wp_seq + 1)}) {
    if (idx < waypoints.size()) {
      const auto & wp = waypoints[idx];
      if (wp.command == 21 || wp.command == 85) {
        active_mode = static_cast<int>(wp.param2);
        RCLCPP_INFO(this->get_logger(),
          "Mission landing detected: command=%d, precision land mode=%d (seq=%d)",
          wp.command, active_mode, (int)idx);
        break;
      }
    }
  }

  if (active_mode == 0) {
    return;
  }

  land_mode_ = active_mode;

  RCLCPP_INFO(this->get_logger(), "AUTO.LAND detected — taking over with OFFBOARD precision landing");
  land_hold_pos_ = vehicle_->get_pos_enu();
  sp_enu_ = vehicle_->get_pos_enu();
  held_yaw_ = GuidanceController::get_yaw(vehicle_->get_q_att());
  guidance_->set_sp_yaw(held_yaw_);
  tag_yaw_abs_.reset();
  yaw_locked_ = false;
  yaw_lock_buf_.clear();
  start_z_sp_ = vehicle_->get_pos_enu().z;
  approach_alt_ = vehicle_->get_pos_enu().z;
  search_cnt_ = 0;
  offboard_activated_ = false;
  target_tracker_->set_target_enu_filtered(std::nullopt);
  target_tracker_->reset_tracking_count();
  transition(PrecLandState::START);
}

void OffboardPreclandController::st_start()
{
  Vector3 hold = land_hold_pos_.has_value() ? land_hold_pos_.value() : vehicle_->get_pos_enu();
  double target_z = std::min(hold.z, target_tracker_->get_virtual_pad_z() + search_alt_);
  double alt = target_tracker_->get_alt(vehicle_->get_pos_enu().z);
  start_z_sp_ = std::max(target_z, start_z_sp_ - guidance_->current_descent_rate(alt, guidance_params_) / ctrl_hz_);
  sp_enu_ = Vector3{hold.x, hold.y, start_z_sp_};

  if (!offboard_activated_) {
    vehicle_->set_mode("OFFBOARD", offboard_activated_);
    offboard_activated_ = true;
    fsm_->search_start.reset();
    RCLCPP_INFO(this->get_logger(), "Requested OFFBOARD mode");
    return;
  }

  if (vehicle_->get_current_mode() != "OFFBOARD") {
    if (target_counter_ % ctrl_hz_ == 0) {
      vehicle_->set_mode("OFFBOARD", offboard_activated_);
    }
    target_counter_++;
    return;
  }

  if (target_tracker_->is_target_fresh(now_sec(), target_timeout_) &&
      target_tracker_->get_tracking_count() >= tracking_confirm_) {
    approach_alt_ = vehicle_->get_pos_enu().z;
    fsm_->align_start = now_sec();
    target_counter_ = 0;
    centered_count_ = 0;
    transition(PrecLandState::HORIZONTAL_APPROACH);
    return;
  }

  if (alt <= search_alt_ + 0.3) {
    if (!fsm_->search_start.has_value()) {
      fsm_->search_start = now_sec();
      RCLCPP_INFO(this->get_logger(), "Reached search altitude (%.1fm). Waiting 5s for target acquisition...", search_alt_);
    }

    double elapsed = now_sec() - fsm_->search_start.value();
    if (elapsed > 5.0) {
      if (land_mode_ == 1) {
        RCLCPP_WARN(this->get_logger(), "Opportunistic mode: Target not found at search altitude → FALLBACK (normal landing)");
        transition(PrecLandState::FALLBACK);
      } else {
        RCLCPP_WARN(this->get_logger(), "Required mode: Target not found at search altitude → active SEARCH");
        fsm_->search_start = now_sec();
        transition(PrecLandState::SEARCH);
      }
    }
  } else {
    fsm_->search_start.reset();
  }
}

void OffboardPreclandController::st_horizontal_approach()
{
  if (!target_tracker_->is_target_fresh(now_sec(), target_timeout_)) {
    RCLCPP_WARN(this->get_logger(), "Target lost during approach");
    fsm_->target_lost_start = now_sec();
    fsm_->target_lost_from = PrecLandState::HORIZONTAL_APPROACH;
    transition(PrecLandState::TARGET_LOST);
    return;
  }

  double servo_gain = target_tracker_->get_servo_gain(vehicle_->get_pos_enu().z, gate_params_);
  auto target_val = target_tracker_->get_target_enu_filtered().has_value() ?
                    target_tracker_->get_target_enu_filtered() : target_tracker_->get_target_enu();

  sp_enu_ = guidance_->calculate_visual_setpoint(vehicle_->get_pos_enu(), target_val, approach_alt_, max_align_step_, servo_gain);

  double ar = target_tracker_->get_align_r(vehicle_->get_pos_enu().z, gate_params_);
  double rel_norm = target_tracker_->get_target_rel_norm();
  if (rel_norm <= ar) {
    centered_count_++;
  } else {
    centered_count_ = 0;
  }

  if (target_counter_ % ctrl_hz_ == 0) {
    RCLCPP_INFO(this->get_logger(), "APPROACH: alt=%.1f err=%.2f gate=%.2f cnt=%d/%d",
      target_tracker_->get_alt(vehicle_->get_pos_enu().z), rel_norm, ar, centered_count_, align_confirm_);
  }
  target_counter_++;

  if (centered_count_ >= align_confirm_) {
    descent_z_sp_ = vehicle_->get_pos_enu().z;
    target_counter_ = 0;
    centered_count_ = 0;
    descent_drift_count_ = 0;
    transition(PrecLandState::DESCEND_ABOVE_TARGET);
    return;
  }

  if (fsm_->align_start.has_value() && (now_sec() - fsm_->align_start.value()) > align_timeout_) {
    double dr = target_tracker_->get_descent_r(vehicle_->get_pos_enu().z, gate_params_);
    if (rel_norm <= dr) {
      descent_z_sp_ = vehicle_->get_pos_enu().z;
      target_counter_ = 0;
      transition(PrecLandState::DESCEND_ABOVE_TARGET);
      return;
    }
    fsm_->align_start = now_sec();
  }
}

void OffboardPreclandController::st_descend_above_target()
{
  if (!target_tracker_->is_target_fresh(now_sec(), target_timeout_)) {
    fsm_->target_lost_start = now_sec();
    fsm_->target_lost_from = PrecLandState::DESCEND_ABOVE_TARGET;
    transition(PrecLandState::TARGET_LOST);
    return;
  }

  double alt = target_tracker_->get_alt(vehicle_->get_pos_enu().z);
  double dr = target_tracker_->get_descent_r(vehicle_->get_pos_enu().z, gate_params_);
  double rel_norm = target_tracker_->get_target_rel_norm();
  bool descent_ok = rel_norm <= dr;

  if (alt < abort_alt_param_ && !descent_ok) {
    double age = now_sec() - target_tracker_->get_last_pose_time();
    if (age <= 0.5 && rel_norm <= low_alt_max_err_) {
      RCLCPP_WARN(this->get_logger(), "Low-alt guarded commit → FINAL_APPROACH");
      transition(PrecLandState::FINAL_APPROACH);
      return;
    }
  }

  if (align_yaw_to_tag_) {
    if (yaw_lock_stage_ == 0 && alt <= yaw_lock_alt_) {
      yaw_lock_stage_ = 1;
      yaw_locked_ = false;
      yaw_lock_buf_.clear();
      yaw_realign_complete_ = false;
      realign_cnt_ = 0;
      fsm_->yaw_lock_stage_start = now_sec();
      RCLCPP_INFO(this->get_logger(), "Entering Stage 1 Yaw Lock at 7m");
    } else if (yaw_lock_stage_ == 1 && yaw_realign_complete_ && alt <= yaw_lock_alt_2_) {
      yaw_lock_stage_ = 2;
      yaw_locked_ = false;
      yaw_lock_buf_.clear();
      yaw_realign_complete_ = false;
      realign_cnt_ = 0;
      fsm_->yaw_lock_stage_start = now_sec();
      RCLCPP_INFO(this->get_logger(), "Entering Stage 2 Yaw Lock at 3m");
    }
  }

  bool in_lock_hover = (align_yaw_to_tag_ && !yaw_realign_complete_ && (yaw_lock_stage_ == 1 || yaw_lock_stage_ == 2));
  double current_yaw_val = GuidanceController::get_yaw(vehicle_->get_q_att());
  double yaw_err = std::abs(GuidanceController::wrap_angle(guidance_->get_sp_yaw() - current_yaw_val));
  bool rotating = (align_yaw_to_tag_ && yaw_locked_ && yaw_err > (3.0 * M_PI / 180.0));

  if (in_lock_hover) {
    double hover_z = (yaw_lock_stage_ == 1) ? yaw_lock_alt_ : yaw_lock_alt_2_;
    descent_z_sp_ = target_tracker_->get_virtual_pad_z() + hover_z;
    descent_drift_count_ = 0;

    double elapsed_hover = now_sec() - fsm_->yaw_lock_stage_start;
    if (elapsed_hover > yaw_lock_timeout_ && !yaw_locked_ && !yaw_realign_complete_) {
      if (yaw_lock_buf_.size() >= static_cast<size_t>(yaw_lock_min_samples_)) {
        tag_yaw_abs_ = guidance_->compute_locked_yaw(yaw_lock_buf_, guidance_->get_sp_yaw(), guidance_params_.tag_yaw_sign, guidance_params_.tag_yaw_offset);
        yaw_locked_ = true;
        RCLCPP_WARN(this->get_logger(), "[YAW-TIMEOUT] Stage %d timed out (%.1fs). Using circular mean of %zu samples: sp_yaw=%.1f deg",
          yaw_lock_stage_, elapsed_hover, yaw_lock_buf_.size(), tag_yaw_abs_.value() * 180.0 / M_PI);
      } else {
        yaw_realign_complete_ = true;
        RCLCPP_WARN(this->get_logger(), "[YAW-TIMEOUT] Stage %d timed out (%.1fs) with insufficient samples (%zu/%d). Skipping yaw realign.",
          yaw_lock_stage_, elapsed_hover, yaw_lock_buf_.size(), yaw_lock_min_samples_);
      }
    }

    if (!yaw_locked_) {
      if (target_counter_ % 15 == 0) {
        RCLCPP_INFO(this->get_logger(), "YAW-SAMPLING [Stage %d] at %.1fm: %d/%d samples",
          yaw_lock_stage_, alt, (int)yaw_lock_buf_.size(), yaw_lock_samples_);
      }
    } else if (rotating) {
      if (target_counter_ % 15 == 0) {
        RCLCPP_INFO(this->get_logger(), "YAW-ALIGN (ROTATING) [Stage %d]: err=%.1f deg — holding XY",
          yaw_lock_stage_, yaw_err * 180.0 / M_PI);
      }
    } else {
      bool xy_centered = rel_norm <= dr;
      if (xy_centered) {
        realign_cnt_++;
        if (realign_cnt_ >= 15) {
          yaw_realign_complete_ = true;
          RCLCPP_INFO(this->get_logger(), "YAW-ALIGN & RE-CENTERING COMPLETE [Stage %d] — continuing descent", yaw_lock_stage_);
        }
      } else {
        realign_cnt_ = 0;
      }

      if (target_counter_ % 15 == 0) {
        RCLCPP_INFO(this->get_logger(), "YAW-ALIGN (RE-CENTERING) [Stage %d]: err=%.2fm (gate=%.2fm), stable_cnt=%d/15",
          yaw_lock_stage_, rel_norm, dr, realign_cnt_);
      }
    }
  } else {
    if (descent_ok) {
      descent_drift_count_ = 0;
      double abs_final_alt = target_tracker_->get_virtual_pad_z() + final_alt_param_;
      descent_z_sp_ = std::max(abs_final_alt, descent_z_sp_ - guidance_->current_descent_rate(alt, guidance_params_) / ctrl_hz_);
    } else {
      descent_drift_count_++;
      descent_z_sp_ = vehicle_->get_pos_enu().z;
      if (target_counter_ % ctrl_hz_ == 0) {
        RCLCPP_WARN(this->get_logger(), "DESCENT Z-LOCK: err=%.2f > gate=%.2f", rel_norm, dr);
      }
      if (descent_drift_count_ >= align_confirm_) {
        bool physically_low = vehicle_->get_pos_enu().z < (final_alt_param_ + 0.5);
        if (alt > abort_alt_param_ && !physically_low) {
          fsm_->search_start = now_sec();
          transition(PrecLandState::SEARCH);
        } else {
          fsm_->align_start = now_sec();
          centered_count_ = 0;
          transition(PrecLandState::HORIZONTAL_APPROACH);
        }
        return;
      }
    }
  }

  double servo_gain = target_tracker_->get_servo_gain(vehicle_->get_pos_enu().z, gate_params_);
  auto target_val = target_tracker_->get_target_enu_filtered().has_value() ?
                    target_tracker_->get_target_enu_filtered() : target_tracker_->get_target_enu();

  if (rotating) {
    sp_enu_.z = descent_z_sp_;
  } else {
    sp_enu_ = guidance_->calculate_visual_setpoint(vehicle_->get_pos_enu(), target_val, descent_z_sp_, max_descent_step_, servo_gain);
  }

  if (target_counter_ % ctrl_hz_ == 0) {
    std::string phase = descent_ok ? "descending" : "z-locked";
    RCLCPP_INFO(this->get_logger(), "DESCEND (%s): alt=%.2f z_sp=%.2f rate=%.2f err=%.2f gate=%.2f",
      phase.c_str(), alt, descent_z_sp_, guidance_->current_descent_rate(alt, guidance_params_), rel_norm, dr);
  }
  target_counter_++;

  if (alt <= final_alt_param_ + 0.15 ||
      vehicle_->get_landed_state() == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND) {
    RCLCPP_INFO(this->get_logger(), "Final altitude or ground contact reached (relative_alt=%.2fm, landed=%d)",
                alt, vehicle_->get_landed_state());
    transition(PrecLandState::FINAL_APPROACH);
  }
}

void OffboardPreclandController::st_final_approach()
{
  double elapsed = now_sec() - fsm_->final_approach_start;
  double actual_drop = final_approach_entry_z_ - vehicle_->get_pos_enu().z;
  double expected_drop = final_descent_rate_ * elapsed;

  if (target_counter_ % ctrl_hz_ == 0) {
    RCLCPP_INFO(this->get_logger(),
      "FINAL_APPROACH: t=%.1fs alt=%.3fm drop=%.3f/%.3fm final_xy=(%.2f,%.2f) landed=%d disarm_req=%s",
      elapsed, vehicle_->get_pos_enu().z, actual_drop, expected_drop, final_x_, final_y_,
      (int)vehicle_->get_landed_state(), disarm_requested_ ? "true" : "false");
  }
  target_counter_++;

  if (disarm_requested_) {
    sp_enu_.z = vehicle_->get_pos_enu().z - 0.2;
    return;
  }

  if (vehicle_->get_landed_state() == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND) {
    RCLCPP_INFO(this->get_logger(), "Ground contact detected via LandedState → force-disarm");
    vehicle_->set_px4_param_float("COM_DISARM_LAND", 0.1f);
    disarm_requested_ = true;
    disarm_attempt_time_first_ = now_sec();
    vehicle_->disarm(disarm_attempt_time_);
    return;
  }

  if (target_tracker_->is_target_fresh(now_sec(), target_timeout_)) {
    auto target_val = target_tracker_->get_target_enu_filtered().has_value() ?
                      target_tracker_->get_target_enu_filtered() : target_tracker_->get_target_enu();
    if (target_val.has_value()) {
      double tx = std::get<0>(target_val.value());
      double ty = std::get<1>(target_val.value());
      double dx = tx - final_x_;
      double dy = ty - final_y_;
      double dist = std::sqrt(dx*dx + dy*dy);
      if (dist > final_align_step_) {
        double scale = final_align_step_ / dist;
        final_x_ += dx * scale;
        final_y_ += dy * scale;
      } else {
        final_x_ = tx;
        final_y_ = ty;
      }
    }
  }

  sp_enu_.x = final_x_;
  sp_enu_.y = final_y_;
  sp_enu_.z = final_approach_entry_z_ - expected_drop;

  if (elapsed >= 1.0 && (expected_drop - actual_drop) > 0.20) {
    RCLCPP_INFO(this->get_logger(), "Ground contact: blocked by %.1fcm → force-disarm", (expected_drop - actual_drop) * 100.0);
    vehicle_->set_px4_param_float("COM_DISARM_LAND", 0.1f);
    disarm_requested_ = true;
    disarm_attempt_time_first_ = now_sec();
    vehicle_->disarm(disarm_attempt_time_);
    return;
  }

  if (elapsed > final_approach_timeout_) {
    RCLCPP_WARN(this->get_logger(), "FINAL_APPROACH timeout (%.1fs) → force-disarm", elapsed);
    vehicle_->set_px4_param_float("COM_DISARM_LAND", 0.1f);
    disarm_requested_ = true;
    disarm_attempt_time_first_ = now_sec();
    vehicle_->disarm(disarm_attempt_time_);
    return;
  }
}

void OffboardPreclandController::st_search()
{
  double s_alt = std::min(search_alt_, search_alt_max_);
  Vector3 anchor;
  auto target_val = target_tracker_->get_target_enu_filtered().has_value() ?
                    target_tracker_->get_target_enu_filtered() : target_tracker_->get_target_enu();
  if (target_val.has_value()) {
    anchor.x = std::get<0>(target_val.value());
    anchor.y = std::get<1>(target_val.value());
  } else if (land_hold_pos_.has_value()) {
    anchor = land_hold_pos_.value();
  } else {
    anchor = vehicle_->get_pos_enu();
  }

  sp_enu_ = Vector3{anchor.x, anchor.y, target_tracker_->get_virtual_pad_z() + s_alt};
  search_cnt_++;

  if (target_tracker_->is_target_fresh(now_sec(), target_timeout_) &&
      target_tracker_->get_tracking_count() >= tracking_confirm_) {
    approach_alt_ = vehicle_->get_pos_enu().z;
    fsm_->align_start = now_sec();
    target_counter_ = 0;
    centered_count_ = 0;
    transition(PrecLandState::HORIZONTAL_APPROACH);
    return;
  }

  if (fsm_->search_start.has_value() && (now_sec() - fsm_->search_start.value()) > search_timeout_) {
    RCLCPP_WARN(this->get_logger(), "Search timeout");
    if (search_cnt_ >= max_search_) {
      transition(PrecLandState::FALLBACK);
    } else {
      fsm_->search_start = now_sec();
    }
  }
}

void OffboardPreclandController::st_target_lost()
{
  if (!fsm_->target_lost_start.has_value()) {
    fsm_->target_lost_start = now_sec();
  }

  if (target_tracker_->is_target_fresh(now_sec(), target_timeout_) &&
      target_tracker_->get_tracking_count() >= tracking_confirm_) {
    PrecLandState resume = fsm_->target_lost_from;
    RCLCPP_INFO(this->get_logger(), "Target reacquired → resuming");
    fsm_->target_lost_start.reset();
    target_counter_ = 0;
    centered_count_ = 0;
    if (resume == PrecLandState::HORIZONTAL_APPROACH) {
      fsm_->align_start = now_sec();
    }
    transition(resume);
    return;
  }

  double elapsed = now_sec() - fsm_->target_lost_start.value();
  sp_enu_ = vehicle_->get_pos_enu();

  if (elapsed > target_loss_grace_) {
    bool physically_low = vehicle_->get_pos_enu().z < (final_alt_param_ + 0.5);
    double alt = target_tracker_->get_alt(vehicle_->get_pos_enu().z);
    if (alt > abort_alt_param_ && !physically_low) {
      fsm_->search_start = now_sec();
      transition(PrecLandState::SEARCH);
    } else {
      RCLCPP_WARN(this->get_logger(), "Target lost near ground → FINAL_APPROACH");
      transition(PrecLandState::FINAL_APPROACH);
    }
  }
}

void OffboardPreclandController::st_fallback()
{
  RCLCPP_WARN(this->get_logger(), "Fallback → reverting to AUTO.LAND (GPS landing)");
  vehicle_->set_mode("AUTO.LAND", offboard_activated_);
  transition(PrecLandState::DONE);
}

void OffboardPreclandController::st_done()
{
  if (!vehicle_->is_armed()) {
    RCLCPP_INFO(this->get_logger(), "LANDING COMPLETE — disarmed");
    transition(PrecLandState::IDLE);
  }
}

}  // namespace precision_landing

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(precision_landing::OffboardPreclandController)
