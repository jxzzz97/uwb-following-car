#include <follow_core.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string>
using namespace follow;
static Measurement measurement(float distance,float angle,uint32_t now,uint32_t seq=1) {
  Measurement m;
  m.distance_cm=distance;m.angle_deg=angle;
  m.left_cm=distance*sinf(angle*.01745329252f);
  m.forward_cm=distance*cosf(angle*.01745329252f);
  m.timestamp_ms=now;m.sequence=seq;m.valid=true;return m;
}
static void feed(UwbParser &p,const std::string &s,uint32_t now) {
  for(char ch:s)p.feed(ch,now);
}
static void parserTests() {
  UwbParser p;
  feed(p,"DIST:200.00, X:0, Y:200, A:0, RSSI:-70\r\n",10);
  assert(p.stats().accepted==1 && p.measurement().timestamp_ms==10);
  for(const char *bad:{"DIST:200, X:nan, Y:200, A:0\n",
      "DIST:200, X:garbage, Y:200, A:0\n","DIST:200, X:0, Y:200, A:\n",
      "DIST:200, X:0, Y:20, A:0\n","DIST:inf, X:0, Y:200, A:0\n"})feed(p,bad,20);
  assert(p.stats().rejected==5 && p.measurement().timestamp_ms==10);
  feed(p,std::string(300,'x')+"DIST:200, X:0, Y:200, A:0\n",30);
  assert(p.stats().accepted==1 && p.stats().overflows==1);
  feed(p,"DIST:450, X:0, Y:450, A:0\n",40);
  assert(p.stats().accepted==1); // Jump rejected while an established link is recent.
  feed(p,"DIST:450, X:0, Y:450, A:0\n",600);
  assert(p.stats().accepted==2 && p.measurement().sequence==2);
  puts("PASS UART format, nonfinite rejection, overflow recovery, jump/reacquisition");
}
static void filterTests() {
  VehicleConfig v;FollowConfig c;TargetFilter f;
  auto first=f.update(measurement(200,10,0),v,c,0,.05f);
  auto held=f.update(measurement(200,20,0),v,c,50,.05f);
  assert(fabsf(first.angle_deg-10)<.01f && fabsf(held.angle_deg-10)<.01f);
  assert(!f.update(measurement(200,0,0),v,c,301,.05f).valid);
  auto rear=f.update(measurement(200,170,350,2),v,c,350,.05f);
  assert(!rear.valid);
  f.reset();v.anchor_left_cm=20;
  auto offset=f.update(measurement(200,0,400,3),v,c,400,.05f);
  assert(offset.angle_deg>5 && offset.angle_deg<6);
  f.reset();v.anchor_left_cm=0;v.anchor_yaw_deg=30;
  auto rotated=f.update(measurement(200,0,450,4),v,c,450,.05f);
  assert(fabsf(rotated.angle_deg-30)<.01f);
  puts("PASS fresh-frame semantics, timeout, field of view, Anchor translation/rotation");
}
static void kinematicsTests() {
  VehicleConfig v;Motion motion;motion.forward_cm_s=30;motion.yaw_deg_s=25;
  auto a=inverseKinematics(motion,v);assert(a.right_rpm>a.left_rpm);
  motion.yaw_deg_s=-25;auto b=inverseKinematics(motion,v);
  assert(fabsf(a.left_rpm-b.right_rpm)<.01f);
  v.wheel_diameter_cm*=2;auto big=inverseKinematics(motion,v);
  assert(fabsf(big.left_rpm*2-b.left_rpm)<.01f);
  v=VehicleConfig{};v.track_width_cm*=1.2f;
  auto wide=inverseKinematics(motion,v);
  assert(wide.left_rpm-wide.right_rpm>b.left_rpm-b.right_rpm);
  motion.forward_cm_s=1000;motion.yaw_deg_s=1000;
  auto limited=inverseKinematics(motion,v);
  const float cap=60*v.max_wheel_speed_cm_s/(3.14159265359f*v.wheel_diameter_cm);
  assert(limited.left_rpm>=0 && limited.right_rpm>=0);
  assert(fmaxf(limited.left_rpm,limited.right_rpm)<=cap+.01f);
  assert(sweptRadiusCm(v)>v.body_width_cm/2);
  FollowConfig c;assert(validConfig(v,c));v.wheel_diameter_cm=0;
  assert(!validConfig(v,c));v=VehicleConfig{};c.target_distance_cm=NAN;
  assert(!validConfig(v,c));
  puts("PASS turn signs, wheel/track resizing, output saturation, configuration validation");
}
static void controllerTests() {
  Controller c;auto cmd=c.update(measurement(250,0,0),0);
  assert(cmd.brake && c.state()==State::Disarmed);
  c.arm(true);cmd=c.update(measurement(250,20,50,2),50);
  assert(c.state()==State::Following && cmd.right_rpm>cmd.left_rpm);
  const float step=35*.05f*60/(3.14159265359f*7.7f);
  assert(cmd.right_rpm<=step+.01f);
  for(uint32_t t=100;t<=350;t+=50)c.update(measurement(250,20,50,2),t);
  cmd=c.update(measurement(250,20,50,2),400);
  assert(cmd.brake && c.state()==State::SignalLost);
  c.arm(true);cmd=c.update(measurement(80,0,450,3),450);
  assert(cmd.brake && c.state()==State::Arrived);
  c.arm(true);cmd=c.update(measurement(128,0,500,4),500);
  assert(cmd.brake); // Hysteresis: initial release requires >135 cm.
  c.arm(true);cmd=c.update(measurement(250,0,550,5),550);
  assert(!cmd.brake);
  cmd=c.update(measurement(90,0,600,6),600);
  assert(cmd.brake); // Raw near-target stop bypasses filter lag.
  c.setOta(true);cmd=c.update(measurement(250,0,650,7),650);
  assert(cmd.brake && !c.armed() && c.state()==State::Ota);
  c.setOta(false);cmd=c.update(measurement(250,0,700,8),700);
  assert(cmd.brake && !c.armed());
  assert(c.setFollowDistance(160));c.arm(true);assert(!c.setFollowDistance(170));
  c.update(measurement(250,0,750,9),750);
  cmd=c.update(measurement(250,0,1000,10),1000);
  assert(cmd.brake && c.state()==State::TimingFault && !c.armed());
  Controller wrap;wrap.arm(true);
  wrap.update(measurement(250,0,0xfffffff0u),0xfffffff0u);
  cmd=wrap.update(measurement(250,0,34,2),34);
  assert(!cmd.brake && wrap.state()==State::Following);
  puts("PASS boot stop, acceleration, loss stop, hysteresis, raw stop, OTA, timing, millis wrap");
}
int main(){parserTests();filterTests();kinematicsTests();controllerTests();puts("All core tests passed.");}
