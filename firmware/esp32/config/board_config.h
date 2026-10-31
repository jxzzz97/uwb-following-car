#pragma once
#include <stdint.h>
// ESP32-WROOM-32 / Waveshare DDSM Driver HAT (A) reference wiring.
constexpr int UWB_RX_PIN = 14, UWB_TX_PIN = -1;
constexpr int MOTOR_TX_PIN = 19, MOTOR_RX_PIN = 18;
constexpr uint32_t UWB_BAUD = 115200, MOTOR_BAUD = 115200;
enum class WheelSide { Left, Right };
struct MotorChannel { uint8_t id; WheelSide side; int8_t direction; float motor_revs_per_wheel_rev; };
// Physical left/right are defined looking forward from the vehicle.
// Assign physical motor IDs before enabling motion; no algorithm-level swap.
constexpr MotorChannel MOTOR_CHANNELS[] = {
  {1, WheelSide::Left, 1, 1}, {2, WheelSide::Right, -1, 1}
};
// Add a second channel per side for four individually driven wheels (unique IDs).
constexpr int MOTOR_PROTOCOL_LIMIT_RPM = 330;
constexpr uint32_t MOTOR_FRAME_GAP_MS = 4;
