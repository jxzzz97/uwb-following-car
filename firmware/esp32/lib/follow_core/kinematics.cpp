#include "follow_core.h"
#include <math.h>
namespace follow {
bool validConfig(const VehicleConfig &v, const FollowConfig &c) {
  const float values[] = {v.body_length_cm,v.body_width_cm,v.track_width_cm,
    v.wheelbase_cm,v.wheel_diameter_cm,v.max_wheel_speed_cm_s,v.max_yaw_rate_deg_s,
    v.min_turn_radius_cm,v.acceleration_cm_s2,v.deceleration_cm_s2,v.effective_track_scale,
    c.target_distance_cm,c.stop_distance_cm,c.resume_distance_cm,c.distance_gain_per_s,
    c.steering_gain_per_s,c.alignment_angle_deg,c.max_target_angle_deg};
  for (float value : values) if (!isfinite(value) || value <= 0) return false;
  return isfinite(v.anchor_left_cm) && isfinite(v.anchor_forward_cm) &&
    isfinite(v.anchor_yaw_deg) && isfinite(c.angle_deadband_deg) &&
    isfinite(c.full_speed_angle_deg) && c.angle_deadband_deg >= 0 &&
    c.angle_deadband_deg < c.full_speed_angle_deg &&
    c.stop_distance_cm <= c.target_distance_cm &&
    c.target_distance_cm < c.resume_distance_cm &&
    c.full_speed_angle_deg < c.alignment_angle_deg &&
    c.alignment_angle_deg < c.max_target_angle_deg && c.max_target_angle_deg < 90 &&
    c.control_period_ms >= 10 && c.control_period_ms <= 100 &&
    c.measurement_timeout_ms >= c.control_period_ms && c.measurement_timeout_ms <= 2000 &&
    c.max_control_gap_ms > c.control_period_ms && c.max_control_gap_ms <= 1000;
}
float sweptRadiusCm(const VehicleConfig &v) {
  return .5f * hypotf(v.body_length_cm, v.body_width_cm);
}
WheelCommand inverseKinematics(Motion m, const VehicleConfig &v) {
  const float track = v.track_width_cm * v.effective_track_scale;
  const float differential = m.yaw_deg_s * .01745329252f * track * .5f;
  // Forward-only arcs: an inner wheel stops, it never reverses automatically.
  float left = fmaxf(0, m.forward_cm_s - differential);
  float right = fmaxf(0, m.forward_cm_s + differential);
  const float peak = fmaxf(left, right);
  if (peak > v.max_wheel_speed_cm_s) {
    const float scale = v.max_wheel_speed_cm_s / peak;
    left *= scale; right *= scale;
  }
  const float to_rpm = 60 / (3.14159265359f * v.wheel_diameter_cm);
  WheelCommand out;
  out.left_rpm = left * to_rpm; out.right_rpm = right * to_rpm;
  out.brake = false;
  return out;
}
} // namespace follow
