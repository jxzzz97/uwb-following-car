#pragma once
#include <follow_core.h>

// Reference chassis dimensions. Measure these values on your own vehicle.
inline follow::VehicleConfig vehicleConfig() {
  follow::VehicleConfig v;
  v.body_length_cm = 60;
  v.body_width_cm = 40;
  v.track_width_cm = 36; // Wheel contact centre to centre, not body width.
  v.wheelbase_cm = 24;
  v.wheel_diameter_cm = 7.7f;
  v.anchor_left_cm = 0;    // Relative to the differential axle reference.
  v.anchor_forward_cm = 0;
  v.anchor_yaw_deg = 0;
  v.max_wheel_speed_cm_s = 60;
  v.max_yaw_rate_deg_s = 70;
  v.min_turn_radius_cm = 30;
  v.effective_track_scale = 1; // Calibrate for four-wheel skid steering.
  v.acceleration_cm_s2 = 35;
  v.deceleration_cm_s2 = 70;
  return v;
}
