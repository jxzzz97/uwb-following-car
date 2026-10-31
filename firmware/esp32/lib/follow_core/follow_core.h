#pragma once
#include <stddef.h>
#include <stdint.h>
#include "ekf_filter.h"

namespace follow {
// Coordinates: +x is left, +y is forward, positive yaw turns left.
struct VehicleConfig {
  float body_length_cm = 60, body_width_cm = 40;
  float track_width_cm = 36, wheelbase_cm = 24, wheel_diameter_cm = 7.7f;
  float anchor_left_cm = 0, anchor_forward_cm = 0, anchor_yaw_deg = 0;
  float max_wheel_speed_cm_s = 60, max_yaw_rate_deg_s = 70;
  float min_turn_radius_cm = 30;
  float acceleration_cm_s2 = 35, deceleration_cm_s2 = 70;
  // Calibrated effective track / measured track for a skid-steer chassis.
  float effective_track_scale = 1;
};
struct FollowConfig {
  float target_distance_cm = 120;
  float stop_distance_cm = 120, resume_distance_cm = 135;
  float distance_gain_per_s = 0.7f, steering_gain_per_s = 1.6f;
  float angle_deadband_deg = 3, full_speed_angle_deg = 7;
  float alignment_angle_deg = 42, max_target_angle_deg = 80;
  uint32_t measurement_timeout_ms = 300;
  uint32_t control_period_ms = 50, max_control_gap_ms = 150;
};
struct Measurement {
  float distance_cm = 0, left_cm = 0, forward_cm = 0, angle_deg = 0;
  uint32_t timestamp_ms = 0, sequence = 0;
  bool valid = false;
};
struct ParserStats {
  uint32_t bytes = 0, lines = 0, accepted = 0, rejected = 0, overflows = 0;
  const char *last_reject = "none";
};
class UwbParser {
 public:
  void feed(char byte, uint32_t now_ms);
  const Measurement &measurement() const { return measurement_; }
  const ParserStats &stats() const { return stats_; }
 private:
  void parse(uint32_t now_ms);
  char line_[256] = {};
  size_t used_ = 0;
  bool dropping_ = false;
  Measurement measurement_;
  ParserStats stats_;
};
struct Target {
  float distance_cm = 0, angle_deg = 0;
  uint32_t timestamp_ms = 0;
  bool valid = false;
};
class TargetFilter {
 public:
  TargetFilter();
  void reset();
  Target update(const Measurement &m, const VehicleConfig &vehicle,
                const FollowConfig &config, uint32_t now_ms, float dt_s);
 private:
  EkfState_t filter_;
  uint32_t last_sequence_ = 0;
};
enum class State { Disarmed, SignalLost, OutOfView, Arrived, Following,
                   Ota, TimingFault, ConfigFault };
const char *stateName(State state);
struct Motion { float forward_cm_s = 0, yaw_deg_s = 0; };
struct WheelCommand {
  float left_rpm = 0, right_rpm = 0;
  bool brake = true;
};
bool validConfig(const VehicleConfig &vehicle, const FollowConfig &config);
// A conservative footprint envelope; pure following does not perform collision tests.
float sweptRadiusCm(const VehicleConfig &vehicle);
WheelCommand inverseKinematics(Motion motion, const VehicleConfig &vehicle);
Motion followDemand(const Target &target, const VehicleConfig &vehicle,
                    const FollowConfig &config);
class Controller {
 public:
  Controller(VehicleConfig vehicle = {}, FollowConfig config = {});
  void arm(bool enabled);
  void setOta(bool active);
  bool setFollowDistance(float cm); // RAM update; application owns persistence.
  WheelCommand update(const Measurement &m, uint32_t now_ms);
  State state() const { return state_; }
  bool armed() const { return armed_; }
  const Target &target() const { return target_; }
  const Motion &demand() const { return demand_; }
  const VehicleConfig &vehicle() const { return vehicle_; }
  const FollowConfig &config() const { return config_; }
 private:
  WheelCommand stop(State state);
  VehicleConfig vehicle_;
  FollowConfig config_;
  TargetFilter filter_;
  Target target_;
  Motion demand_;
  WheelCommand previous_;
  State state_ = State::Disarmed;
  bool armed_ = false, ota_ = false, arrived_ = false, have_tick_ = false;
  uint32_t last_tick_ms_ = 0;
};
} // namespace follow
