#include "app_runtime.h"
#include "motor_driver.h"
#include "local_settings.h"
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
static bool services_started=false;
void networkBegin() {
  if(!strlen(FOLLOW_WIFI_SSID))return;
  WiFi.mode(WIFI_STA);WiFi.setHostname(FOLLOW_HOSTNAME);
  WiFi.begin(FOLLOW_WIFI_SSID,FOLLOW_WIFI_PASSWORD);
}
void networkLoop() {
  if(!services_started && WiFi.status()==WL_CONNECTED) {
    webBegin();
    if(strlen(FOLLOW_OTA_PASSWORD)>=16) {
      ArduinoOTA.setHostname(FOLLOW_HOSTNAME);ArduinoOTA.setPassword(FOLLOW_OTA_PASSWORD);
      ArduinoOTA.onStart([]{app().ota_active=true;app().controller.setOta(true);motorStop();});
      ArduinoOTA.onEnd([]{motorStop();});
      ArduinoOTA.onError([](ota_error_t){app().ota_active=false;app().controller.setOta(false);motorStop();});
      ArduinoOTA.begin();
    }
    services_started=true;
    Serial.printf("[NET] connected: %s\n",WiFi.localIP().toString().c_str());
  }
  if(services_started) {
    if(strlen(FOLLOW_OTA_PASSWORD)>=16)ArduinoOTA.handle();
    webLoop();
  }
}
