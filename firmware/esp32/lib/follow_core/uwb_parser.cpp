#include "follow_core.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace follow {
// strtof end pointers distinguish a real zero from malformed or empty text.
static bool field(const char *line, const char *key, float &value) {
  const char *p = strstr(line, key);
  if (!p) return false;
  p += strlen(key);
  char *end = nullptr;
  value = strtof(p, &end);
  if (end == p || !isfinite(value)) return false;
  while (*end == ' ' || *end == '\t') ++end;
  return *end == ',' || *end == '\0';
}
void UwbParser::feed(char byte, uint32_t now_ms) {
  ++stats_.bytes;
  if (byte == '\r') return;
  if (byte == '\n') {
    ++stats_.lines;
    if (!dropping_ && used_) { line_[used_] = 0; parse(now_ms); }
    used_ = 0;
    dropping_ = false;
    return;
  }
  if (dropping_) return;
  if (used_ >= sizeof(line_) - 1 || byte == '\0') {
    dropping_ = true;
    ++stats_.overflows;
    ++stats_.rejected;
    stats_.last_reject = "line overflow or NUL";
    return;
  }
  line_[used_++] = byte;
}
void UwbParser::parse(uint32_t now_ms) {
  Measurement candidate;
  const char *reason = nullptr;
  if (strncmp(line_, "DIST:", 5) != 0 ||
      !field(line_, "DIST:", candidate.distance_cm) ||
      !field(line_, ", X:", candidate.left_cm) ||
      !field(line_, ", Y:", candidate.forward_cm) ||
      !field(line_, ", A:", candidate.angle_deg)) reason = "format or nonfinite";
  else if (candidate.distance_cm <= 0 || candidate.distance_cm > 5000 ||
           fabsf(candidate.left_cm) > 5000 || fabsf(candidate.forward_cm) > 5000 ||
           fabsf(candidate.angle_deg) > 180) reason = "range";
  else {
    const float norm = hypotf(candidate.left_cm, candidate.forward_cm);
    if (fabsf(norm - candidate.distance_cm) > fmaxf(30, candidate.distance_cm * .15f))
      reason = "distance/xy mismatch";
    // Retain the original parser's jump gate and 500 ms reacquisition window.
    else if (measurement_.valid && uint32_t(now_ms - measurement_.timestamp_ms) <= 500 &&
             (fabsf(candidate.distance_cm - measurement_.distance_cm) > 200 ||
              fabsf(candidate.angle_deg - measurement_.angle_deg) > 60)) reason = "jump";
  }
  if (reason) { ++stats_.rejected; stats_.last_reject = reason; return; }
  candidate.timestamp_ms = now_ms;
  candidate.sequence = measurement_.sequence + 1;
  candidate.valid = true;
  measurement_ = candidate;
  ++stats_.accepted;
  stats_.last_reject = "none";
}
} // namespace follow
