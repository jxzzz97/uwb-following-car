#pragma once
#include <follow_core.h>
struct AppRuntime {
  follow::UwbParser parser;
  follow::Controller controller;
  bool driver_ready = false, ota_active = false;
  uint32_t control_interval_ms = 0;
};
AppRuntime &app();
void disarm();
void networkBegin();
void networkLoop();
void webBegin();
void webLoop();
void recordLog(uint32_t now_ms);
void loadParameters();
bool saveFollowDistance(float cm);
