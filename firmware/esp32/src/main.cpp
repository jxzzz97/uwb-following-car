#include <Arduino.h>
#include "app_runtime.h"
#include "vehicle_config.h"
#include "follow_config.h"
#include "board_config.h"
#include "motor_driver.h"
static HardwareSerial uwb(2);
static uint32_t last_tick=0;
AppRuntime &app() {
  static AppRuntime runtime;
  return runtime;
}
void disarm() { app().controller.arm(false); motorStop(); }
static void serialControl() {
  static char line[32];static size_t used=0;static bool dropping=false;
  while(Serial.available()) {
    const char ch=char(Serial.read());if(ch=='\r')continue;
    if(ch=='\n') {
      line[used]=0;
      if(!dropping) {
        if(strcmp(line,"stop")==0)disarm();
        else if(strcmp(line,"arm")==0 && app().driver_ready && !app().ota_active)app().controller.arm(true);
      }
      used=0;dropping=false;
    }else if(used<sizeof(line)-1 && !dropping)line[used++]=ch;
    else dropping=true;
  }
}
void setup() {
  Serial.begin(115200);
  app().controller=follow::Controller(vehicleConfig(),followConfig());
  loadParameters();
  app().driver_ready=motorBegin(app().controller.vehicle()) && follow::validConfig(app().controller.vehicle(),app().controller.config());
  uwb.setRxBufferSize(1024);uwb.begin(UWB_BAUD,SERIAL_8N1,UWB_RX_PIN,UWB_TX_PIN);
  networkBegin();
  Serial.println("[INIT] stopped; local commands: arm / stop (newline terminated)");
  Serial.printf("[INIT] driver=%s, footprint swept radius=%.1f cm\n",app().driver_ready?"ready":"configuration rejected",follow::sweptRadiusCm(app().controller.vehicle()));
  last_tick=millis();
}
void loop() {
  while(uwb.available())app().parser.feed(char(uwb.read()),millis());
  serialControl();
  const uint32_t now=millis();
  if(uint32_t(now-last_tick)>=app().controller.config().control_period_ms) {
    app().control_interval_ms=uint32_t(now-last_tick);last_tick=now;
    const auto command=app().controller.update(app().parser.measurement(),now);
    if(!app().driver_ready || app().ota_active)motorStop();else motorApply(command);
    recordLog(now);
  }
  networkLoop();delay(1);
}
