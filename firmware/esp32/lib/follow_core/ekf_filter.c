/**
 * @file    ekf_filter.c
 * @brief   4-state linear Kalman filter (与 AT32 版字节级一致，纯算法无平台依赖)
 */
#include "ekf_filter.h"
#include <string.h>

#define DEFAULT_Q_ANGLE      2.0f
#define DEFAULT_Q_DIST       10.0f
#define DEFAULT_Q_ANGLE_RATE 8.0f
#define DEFAULT_Q_DIST_RATE  40.0f

#define DEFAULT_R_ANGLE      16.0f
#define DEFAULT_R_DIST       36.0f

#define INIT_P_ANGLE         100.0f
#define INIT_P_DIST          400.0f
#define INIT_P_ANGLE_RATE    100.0f
#define INIT_P_DIST_RATE     400.0f

void EKF_Init(EkfState_t *ekf, float dt_sec) {
  memset(ekf, 0, sizeof(EkfState_t));
  ekf->dt = dt_sec;
  ekf->Q[0] = DEFAULT_Q_ANGLE;
  ekf->Q[1] = DEFAULT_Q_DIST;
  ekf->Q[2] = DEFAULT_Q_ANGLE_RATE;
  ekf->Q[3] = DEFAULT_Q_DIST_RATE;
  ekf->R[0] = DEFAULT_R_ANGLE;
  ekf->R[1] = DEFAULT_R_DIST;
  ekf->initialized = 0;
  ekf->rejectCount = 0;
}

void EKF_Seed(EkfState_t *ekf, float angle_deg, float dist_cm) {
  ekf->x[0] = angle_deg;
  ekf->x[1] = dist_cm;
  ekf->x[2] = 0.0f;
  ekf->x[3] = 0.0f;
  memset(ekf->P, 0, sizeof(ekf->P));
  ekf->P[0][0] = INIT_P_ANGLE;
  ekf->P[1][1] = INIT_P_DIST;
  ekf->P[2][2] = INIT_P_ANGLE_RATE;
  ekf->P[3][3] = INIT_P_DIST_RATE;
  ekf->initialized = 1;
  ekf->rejectCount = 0;
}

void EKF_Predict(EkfState_t *ekf) {
  float dt = ekf->dt;
  float *x = ekf->x;
  float (*P)[EKF_N] = ekf->P;
  float P_new[EKF_N][EKF_N];
  float p00, p01, p02, p03;
  float p10, p11, p12, p13;
  float p20, p21, p22, p23;
  float p30, p31, p32, p33;

  if (!ekf->initialized) return;

  x[0] += x[2] * dt;
  x[1] += x[3] * dt;

  p00 = P[0][0]; p01 = P[0][1]; p02 = P[0][2]; p03 = P[0][3];
  p10 = P[1][0]; p11 = P[1][1]; p12 = P[1][2]; p13 = P[1][3];
  p20 = P[2][0]; p21 = P[2][1]; p22 = P[2][2]; p23 = P[2][3];
  p30 = P[3][0]; p31 = P[3][1]; p32 = P[3][2]; p33 = P[3][3];

  P_new[0][0] = p00 + dt * (p20 + p02) + dt * dt * p22 + ekf->Q[0];
  P_new[0][1] = p01 + dt * (p21 + p03) + dt * dt * p23;
  P_new[0][2] = p02 + dt * p22;
  P_new[0][3] = p03 + dt * p23;

  P_new[1][0] = p10 + dt * (p30 + p12) + dt * dt * p32;
  P_new[1][1] = p11 + dt * (p31 + p13) + dt * dt * p33 + ekf->Q[1];
  P_new[1][2] = p12 + dt * p32;
  P_new[1][3] = p13 + dt * p33;

  P_new[2][0] = p20 + dt * p22;
  P_new[2][1] = p21 + dt * p23;
  P_new[2][2] = p22 + ekf->Q[2];
  P_new[2][3] = p23;

  P_new[3][0] = p30 + dt * p32;
  P_new[3][1] = p31 + dt * p33;
  P_new[3][2] = p32;
  P_new[3][3] = p33 + ekf->Q[3];

  memcpy(P, P_new, sizeof(P_new));
}

void EKF_Update(EkfState_t *ekf, float angle_deg, float dist_cm) {
  float (*P)[EKF_N] = ekf->P;
  float *x = ekf->x;

  float y0, y1;
  float S00, S01, S11;
  float det, invDet;
  float Si00, Si01, Si11;
  float K[EKF_N][EKF_M];
  float P_copy[EKF_N][EKF_N];
  int i, j;

  if (!ekf->initialized) {
    EKF_Seed(ekf, angle_deg, dist_cm);
    return;
  }

  y0 = angle_deg - x[0];
  y1 = dist_cm   - x[1];

  S00 = P[0][0] + ekf->R[0];
  S01 = P[0][1];
  S11 = P[1][1] + ekf->R[1];

  det = S00 * S11 - S01 * S01;
  if (det < 1e-6f && det > -1e-6f) return;
  invDet = 1.0f / det;
  Si00 =  S11 * invDet;
  Si01 = -S01 * invDet;
  Si11 =  S00 * invDet;

  {
    float nis = y0 * (Si00 * y0 + Si01 * y1)
              + y1 * (Si01 * y0 + Si11 * y1);
    if (nis > EKF_NIS_GATE) {
      ekf->rejectCount++;
      if (ekf->rejectCount >= EKF_NIS_MAX_REJECT) {
        EKF_Seed(ekf, angle_deg, dist_cm);
      }
      return;
    }
    ekf->rejectCount = 0;
  }

  for (i = 0; i < EKF_N; i++) {
    K[i][0] = P[i][0] * Si00 + P[i][1] * Si01;
    K[i][1] = P[i][0] * Si01 + P[i][1] * Si11;
  }

  for (i = 0; i < EKF_N; i++) {
    x[i] += K[i][0] * y0 + K[i][1] * y1;
  }

  memcpy(P_copy, P, sizeof(P_copy));
  for (i = 0; i < EKF_N; i++) {
    for (j = 0; j < EKF_N; j++) {
      P[i][j] = P_copy[i][j] - K[i][0] * P_copy[0][j]
                              - K[i][1] * P_copy[1][j];
    }
  }
  for (i = 0; i < EKF_N; i++) {
    for (j = i + 1; j < EKF_N; j++) {
      float avg = 0.5f * (P[i][j] + P[j][i]);
      P[i][j] = avg;
      P[j][i] = avg;
    }
  }
}

float EKF_GetAngle(const EkfState_t *ekf) { return ekf->x[0]; }
float EKF_GetDistance(const EkfState_t *ekf) { return ekf->x[1] > 0.0f ? ekf->x[1] : 0.0f; }
float EKF_GetAngleRate(const EkfState_t *ekf) { return ekf->x[2]; }
float EKF_GetDistRate(const EkfState_t *ekf) { return ekf->x[3]; }
