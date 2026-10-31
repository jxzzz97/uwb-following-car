#include "motor_driver.h"
#include "board_config.h"
#include <Arduino.h>
#include <math.h>

static HardwareSerial bus(1);
static MotorOutput output;
static bool ready = false;
static uint8_t crc8(const uint8_t *p, size_t n) {
  uint8_t crc = 0;
  while (n--) { crc ^= *p++; for (int i=0;i<8;++i) crc = (crc>>1) ^ ((crc&1)?0x8c:0); }
  return crc;
}
static void send(uint8_t *packet) {
  bus.write(packet,10); bus.flush(); ++output.packets;
  delay(MOTOR_FRAME_GAP_MS);
}
static void drive(const MotorChannel &channel, float wheel_rpm, bool brake) {
  // Reject nonfinite values at the device boundary, independently of the controller.
  if (!isfinite(wheel_rpm)) { wheel_rpm = 0; brake = true; }
  const float value = wheel_rpm * channel.motor_revs_per_wheel_rev * channel.direction;
  const int16_t rpm = int16_t(lroundf(fmaxf(-MOTOR_PROTOCOL_LIMIT_RPM,
                                   fminf(MOTOR_PROTOCOL_LIMIT_RPM,value))));
  const uint16_t encoded = uint16_t(rpm);
  uint8_t packet[10] = {channel.id,0x64,uint8_t(encoded>>8),uint8_t(encoded),
                         0,0,0,uint8_t(brake?0xff:0),0,0};
  packet[9] = crc8(packet,9); send(packet);
}
bool motorBegin(const follow::VehicleConfig &v) {
  bus.begin(MOTOR_BAUD,SERIAL_8N1,MOTOR_RX_PIN,MOTOR_TX_PIN);
  const size_t count = sizeof(MOTOR_CHANNELS)/sizeof(MOTOR_CHANNELS[0]);
  ready = count >= 2 && count <= 4;
  bool left = false, right = false;
  const float max_wheel_rpm = 60*v.max_wheel_speed_cm_s/(3.14159265359f*v.wheel_diameter_cm);
  for (size_t i=0;i<count;++i) {
    const auto &c = MOTOR_CHANNELS[i];
    left |= c.side == WheelSide::Left; right |= c.side == WheelSide::Right;
    ready &= c.id >= 1 && c.id <= 63 && (c.direction==1 || c.direction==-1) &&
      isfinite(c.motor_revs_per_wheel_rev) && c.motor_revs_per_wheel_rev > 0 &&
      max_wheel_rpm*c.motor_revs_per_wheel_rev <= MOTOR_PROTOCOL_LIMIT_RPM;
    for (size_t j=0;j<i;++j) ready &= c.id != MOTOR_CHANNELS[j].id;
  }
  ready &= left && right;
  // Attempt zero-speed stop even for a rejected configuration.
  for (const auto &c : MOTOR_CHANNELS) drive(c,0,true);
  if (!ready) return false;
  for (int repeat=0;repeat<5;++repeat) for (const auto &c : MOTOR_CHANNELS) {
    uint8_t packet[10] = {c.id,0xa0,0,0,0,0,0,0,0,2}; send(packet);
  }
  motorStop(); return true;
}
void motorStop() {
  for (const auto &c : MOTOR_CHANNELS) drive(c,0,true);
  output.left_rpm = output.right_rpm = 0;
}
void motorApply(const follow::WheelCommand &command) {
  if (!ready || command.brake) { motorStop(); return; }
  for (const auto &c : MOTOR_CHANNELS)
    drive(c,c.side==WheelSide::Left?command.left_rpm:command.right_rpm,false);
  output.left_rpm = command.left_rpm; output.right_rpm = command.right_rpm;
}
const MotorOutput &motorOutput() { return output; }
