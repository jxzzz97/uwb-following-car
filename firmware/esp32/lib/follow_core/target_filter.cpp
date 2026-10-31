#include "follow_core.h"
#include <math.h>
namespace follow {
static constexpr float kDegToRad = 0.017453292519943295f;
TargetFilter::TargetFilter() { reset(); }
void TargetFilter::reset() { EKF_Init(&filter_, .05f); last_sequence_ = 0; }
Target TargetFilter::update(const Measurement &m, const VehicleConfig &v,
                            const FollowConfig &c, uint32_t now, float dt) {
  Target out;
  if (!m.valid || uint32_t(now - m.timestamp_ms) > c.measurement_timeout_ms) {
    reset(); return out;
  }
  // Rotate the Anchor coordinate system, then translate to the axle reference.
  const float yaw = v.anchor_yaw_deg * kDegToRad;
  const float left = m.left_cm * cosf(yaw) + m.forward_cm * sinf(yaw) + v.anchor_left_cm;
  const float forward = m.forward_cm * cosf(yaw) - m.left_cm * sinf(yaw) + v.anchor_forward_cm;
  const float distance = hypotf(left, forward);
  const float angle = atan2f(left, forward) / kDegToRad;
  if (forward <= 0 || fabsf(angle) > c.max_target_angle_deg) {
    reset(); return out;
  }
  filter_.dt = dt;
  EKF_Predict(&filter_);
  // A held UART frame is predicted, never counted as another measurement.
  if (!filter_.initialized || m.sequence != last_sequence_) {
    EKF_Update(&filter_, angle, distance);
    last_sequence_ = m.sequence;
  }
  out.distance_cm = EKF_GetDistance(&filter_);
  out.angle_deg = EKF_GetAngle(&filter_);
  out.timestamp_ms = m.timestamp_ms;
  out.valid = isfinite(out.distance_cm) && isfinite(out.angle_deg);
  return out;
}
} // namespace follow
