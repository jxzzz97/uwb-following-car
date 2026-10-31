#include "app_runtime.h"
#include "motor_driver.h"
#include "local_settings.h"
#include "dashboard.h"
#include <Arduino.h>
#include <WebServer.h>
#include <math.h>
static WebServer server(80);
static uint32_t log_sequence=0;
struct LogRow {
  uint32_t tick, sequence, age;
  float raw_d,raw_a,filtered_d,filtered_a,left,right;
  follow::State state;
};
static LogRow log_rows[256];
static size_t log_count = 0, log_next = 0;

static String number(float value) { return isfinite(value)?String(value,3):String("null"); }
static bool authorize() {
  if (strlen(FOLLOW_CONTROL_TOKEN)<16 || server.header("X-Control-Token")!=FOLLOW_CONTROL_TOKEN) {
    server.send(403,"application/json","{\"message\":\"控制令牌未配置或不正确\"}"); return false;
  }
  return true;
}
static void status() {
  const auto &m=app().parser.measurement(); const auto &t=app().controller.target();
  const auto &d=app().controller.demand(); const auto &s=app().parser.stats(); const auto &o=motorOutput();
  String json; json.reserve(850);
  json="{\"state\":\""+String(follow::stateName(app().controller.state()))+"\",\"armed\":";
  json+=app().controller.armed()?"true":"false";
  json+=",\"driver_ready\":"; json+=app().driver_ready?"true":"false";
  json+=",\"measurement_age_ms\":"; json+=m.valid?String(uint32_t(millis()-m.timestamp_ms)):String("null");
  json+=",\"control_interval_ms\":"+String(app().control_interval_ms);
  json+=",\"raw_distance_cm\":"+(m.valid?number(m.distance_cm):String("null"));
  json+=",\"raw_angle_deg\":"+(m.valid?number(m.angle_deg):String("null"));
  json+=",\"filtered_distance_cm\":"+(t.valid?number(t.distance_cm):String("null"));
  json+=",\"filtered_angle_deg\":"+(t.valid?number(t.angle_deg):String("null"));
  json+=",\"forward_cm_s\":"+number(d.forward_cm_s)+",\"yaw_deg_s\":"+number(d.yaw_deg_s);
  json+=",\"left_command_rpm\":"+number(o.left_rpm)+",\"right_command_rpm\":"+number(o.right_rpm);
  json+=",\"parse_ok\":"+String(s.accepted)+",\"parse_rejected\":"+String(s.rejected);
  json+=",\"last_reject\":\""+String(s.last_reject)+"\",\"motor_packets\":"+String(o.packets);
  json+=",\"target_distance_cm\":"+number(app().controller.config().target_distance_cm)+"}";
  server.sendHeader("Cache-Control","no-store"); server.send(200,"application/json; charset=utf-8",json);
}
static void downloadLog() {
  // Export only when stopped: synchronous transfer must not block a moving chassis.
  if (app().controller.armed()) { server.send(409,"text/plain; charset=utf-8","请先停止跟随再导出日志"); return; }
  server.sendHeader("Content-Disposition","attachment; filename=follow-log.csv");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN); server.send(200,"text/csv; charset=utf-8","");
  server.sendContent("tick_ms,sequence,measurement_age_ms,raw_distance_cm,raw_angle_deg,filtered_distance_cm,filtered_angle_deg,left_command_rpm,right_command_rpm,state\n");
  const size_t start=(log_next+256-log_count)%256;
  char line[240];
  for(size_t i=0;i<log_count;++i) {
    const auto &r=log_rows[(start+i)%256];
    snprintf(line,sizeof(line),"%lu,%lu,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%s\n",
      (unsigned long)r.tick,(unsigned long)r.sequence,(unsigned long)r.age,
      r.raw_d,r.raw_a,r.filtered_d,r.filtered_a,r.left,r.right,follow::stateName(r.state));
    server.sendContent(line);
  }
  server.sendContent("");
}
void webBegin() {
  const char *headers[]={"X-Control-Token"}; server.collectHeaders(headers,1);
  server.on("/",HTTP_GET,[]{ server.send_P(200,"text/html; charset=utf-8",DASHBOARD); });
  server.on("/api/status",HTTP_GET,status);
  server.on("/api/log.csv",HTTP_GET,downloadLog);
  server.on("/api/control",HTTP_POST,[]{
    if(!authorize())return;
    const String action=server.arg("action");
    if(action=="stop")disarm();
    else if(action=="arm" && app().driver_ready && !app().ota_active)app().controller.arm(true);
    else { server.send(409,"application/json","{\"message\":\"请求无效或设备未就绪\"}");return; }
    server.send(200,"application/json","{\"message\":\"操作已执行\"}");
  });
  server.on("/api/params",HTTP_POST,[]{
    if(!authorize())return;
    String value=server.arg("target_distance_cm");char *end=nullptr;
    const float cm=strtof(value.c_str(),&end);
    if(end==value.c_str() || *end || !saveFollowDistance(cm)) {
      server.send(409,"application/json","{\"message\":\"请停车后设置 40–1000 cm 的目标距离\"}");return;
    }
    server.send(200,"application/json","{\"message\":\"已保存至设备，重启后继续生效\"}");
  });
  server.begin();
}
void webLoop() { server.handleClient(); }
void recordLog(uint32_t now) {
  const auto &m=app().parser.measurement();const auto &t=app().controller.target();const auto &o=motorOutput();
  LogRow row{now,++log_sequence,m.valid?uint32_t(now-m.timestamp_ms):UINT32_MAX,
    m.valid?m.distance_cm:NAN,m.valid?m.angle_deg:NAN,
    t.valid?t.distance_cm:NAN,t.valid?t.angle_deg:NAN,o.left_rpm,o.right_rpm,app().controller.state()};
  log_rows[log_next]=row;log_next=(log_next+1)%256;if(log_count<256)++log_count;
  if(log_sequence%4==0)Serial.printf("[FOLLOW] state=%s age=%lu L=%.1f R=%.1f rpm\n",
    follow::stateName(app().controller.state()),(unsigned long)row.age,row.left,row.right);
}
