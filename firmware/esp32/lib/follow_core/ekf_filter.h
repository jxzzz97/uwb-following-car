/**
 * @file    ekf_filter.h
 * @brief   4-state linear Kalman filter (从 AT32 工程原样复用)
 */
#ifndef __EKF_FILTER_H
#define __EKF_FILTER_H

#include <stdint.h>

#define EKF_N 4
#define EKF_M 2

#define EKF_NIS_GATE         16.0f
#define EKF_NIS_MAX_REJECT   5u

typedef struct {
  float x[EKF_N];
  float P[EKF_N][EKF_N];
  float Q[EKF_N];
  float R[EKF_M];
  float dt;
  uint8_t initialized;
  uint8_t rejectCount;
} EkfState_t;

#ifdef __cplusplus
extern "C" {
#endif

void EKF_Init(EkfState_t *ekf, float dt_sec);
void EKF_Predict(EkfState_t *ekf);
void EKF_Update(EkfState_t *ekf, float angle_deg, float dist_cm);
void EKF_Seed(EkfState_t *ekf, float angle_deg, float dist_cm);
float EKF_GetAngle(const EkfState_t *ekf);
float EKF_GetDistance(const EkfState_t *ekf);
float EKF_GetAngleRate(const EkfState_t *ekf);
float EKF_GetDistRate(const EkfState_t *ekf);

#ifdef __cplusplus
}
#endif

#endif /* __EKF_FILTER_H */
