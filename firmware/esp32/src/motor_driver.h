#pragma once
#include <follow_core.h>
// Adapter contract: wheel RPM is logical, forward-positive; driver owns IDs/signs.
bool motorBegin(const follow::VehicleConfig &vehicle);
void motorApply(const follow::WheelCommand &command);
void motorStop();
struct MotorOutput { float left_rpm = 0, right_rpm = 0; uint32_t packets = 0; };
const MotorOutput &motorOutput();
