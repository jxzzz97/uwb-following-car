#include "app_runtime.h"
#include <Preferences.h>
static Preferences storage;
static bool storage_ready = false;
void loadParameters() {
  storage_ready=storage.begin("uwb-follow-v1",false);
  if(storage_ready)app().controller.setFollowDistance(storage.getFloat("distance_cm",app().controller.config().target_distance_cm));
}
bool saveFollowDistance(float cm) {
  if(!storage_ready || app().controller.armed())return false;
  const float previous=app().controller.config().target_distance_cm;
  if(!app().controller.setFollowDistance(cm))return false;
  if(storage.putFloat("distance_cm",cm)==sizeof(float))return true;
  app().controller.setFollowDistance(previous);return false;
}
