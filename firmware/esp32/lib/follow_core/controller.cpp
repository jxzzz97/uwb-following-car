#include "follow_core.h"
#include <math.h>
namespace follow {
static float clamp(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }
const char *stateName(State s) {
  switch (s) {
    case State::Disarmed:return "disarmed"; case State::SignalLost:return "signal_lost";
    case State::OutOfView:return "out_of_view"; case State::Arrived:return "arrived";
    case State::Following:return "following"; case State::Ota:return "ota";
    case State::TimingFault:return "timing_fault"; case State::ConfigFault:return "config_fault";
  }
  return "unknown";
}
Motion followDemand(const Target &t, const VehicleConfig &v, const FollowConfig &c) {
  Motion out;
  const float error = fmaxf(0, t.distance_cm - c.target_distance_cm);
  out.forward_cm_s = fminf(v.max_wheel_speed_cm_s, error * c.distance_gain_per_s);
  const float cruise = out.forward_cm_s;
  const float angle = fabsf(t.angle_deg);
  float steer = fmaxf(0, angle - c.angle_deadband_deg);
  if (t.angle_deg < 0) steer = -steer;
  const float yaw_limit = fminf(v.max_yaw_rate_deg_s,
      cruise / v.min_turn_radius_cm / .01745329252f);
  out.yaw_deg_s = clamp(steer * c.steering_gain_per_s,-yaw_limit,yaw_limit);
  // As in the original pure follower, large angles progressively reduce forward speed.
  const float scale = clamp((c.alignment_angle_deg - angle) /
      (c.alignment_angle_deg - c.full_speed_angle_deg), 0, 1);
  out.forward_cm_s *= scale;
  // Maintain a forward arc at large angles without an on-the-spot reversal.
  if (fabsf(out.yaw_deg_s) > 0 && error > 0) {
    out.forward_cm_s = fmaxf(out.forward_cm_s,
        fabsf(out.yaw_deg_s) * .01745329252f * v.min_turn_radius_cm);
  } else if (error <= 0) out.yaw_deg_s = 0;
  return out;
}
Controller::Controller(VehicleConfig v, FollowConfig c) : vehicle_(v), config_(c) {}
void Controller::arm(bool enabled) {
  armed_ = enabled && validConfig(vehicle_,config_) && !ota_;
  arrived_ = true; have_tick_ = false; filter_.reset();
  target_ = {}; demand_ = {}; previous_ = {};
  state_ = validConfig(vehicle_,config_) ? State::Disarmed : State::ConfigFault;
}
void Controller::setOta(bool active) { ota_ = active; arm(false); }
bool Controller::setFollowDistance(float cm) {
  if (armed_ || !isfinite(cm) || cm < 40 || cm > 1000) return false;
  const float stop_offset = config_.target_distance_cm - config_.stop_distance_cm;
  const float resume_offset = config_.resume_distance_cm - config_.target_distance_cm;
  if (cm - stop_offset <= 0) return false;
  config_.stop_distance_cm = cm - stop_offset;
  config_.resume_distance_cm = cm + resume_offset;
  config_.target_distance_cm = cm;
  return true;
}
WheelCommand Controller::stop(State state) {
  state_ = state; previous_ = {}; demand_ = {};
  return previous_;
}
WheelCommand Controller::update(const Measurement &m, uint32_t now) {
  const uint32_t elapsed = have_tick_ ? uint32_t(now - last_tick_ms_) : config_.control_period_ms;
  last_tick_ms_ = now; have_tick_ = true;
  if (!validConfig(vehicle_,config_)) return stop(State::ConfigFault);
  if (ota_) return stop(State::Ota);
  if (armed_ && elapsed > config_.max_control_gap_ms) {
    armed_ = false; filter_.reset(); target_ = {}; return stop(State::TimingFault);
  }
  const float dt = float(elapsed) * .001f;
  target_ = filter_.update(m, vehicle_, config_, now, dt);
  if (!armed_) return stop(State::Disarmed);
  if (!m.valid || uint32_t(now - m.timestamp_ms) > config_.measurement_timeout_ms) {
    arrived_ = true; return stop(State::SignalLost);
  }
  if (!target_.valid) { arrived_ = true; return stop(State::OutOfView); }
  // Raw and filtered distances both participate in near-target stopping.
  const float yaw = vehicle_.anchor_yaw_deg * .01745329252f;
  const float raw_distance = hypotf(m.left_cm*cosf(yaw) + m.forward_cm*sinf(yaw) + vehicle_.anchor_left_cm,
      m.forward_cm*cosf(yaw) - m.left_cm*sinf(yaw) + vehicle_.anchor_forward_cm);
  if (fminf(raw_distance,target_.distance_cm) <= config_.stop_distance_cm) arrived_ = true;
  if (arrived_ && fminf(raw_distance,target_.distance_cm) > config_.resume_distance_cm) arrived_ = false;
  if (arrived_ || target_.distance_cm <= config_.target_distance_cm) {
    arrived_ = true; return stop(State::Arrived);
  }
  demand_ = followDemand(target_,vehicle_,config_);
  WheelCommand command = inverseKinematics(demand_,vehicle_);
  const float cm_s_to_rpm = 60 / (3.14159265359f * vehicle_.wheel_diameter_cm);
  const float up = vehicle_.acceleration_cm_s2 * dt * cm_s_to_rpm;
  const float down = vehicle_.deceleration_cm_s2 * dt * cm_s_to_rpm;
  const float dl = command.left_rpm - previous_.left_rpm;
  const float dr = command.right_rpm - previous_.right_rpm;
  // A shared interpolation factor retains the intended wheel ratio at launch.
  float fraction = 1;
  if (fabsf(dl)>0) fraction=fminf(fraction,(dl>0?up:down)/fabsf(dl));
  if (fabsf(dr)>0) fraction=fminf(fraction,(dr>0?up:down)/fabsf(dr));
  command.left_rpm = previous_.left_rpm + dl*fraction;
  command.right_rpm = previous_.right_rpm + dr*fraction;
  previous_ = command; state_ = State::Following;
  return command;
}
} // namespace follow
