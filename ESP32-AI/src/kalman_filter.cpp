#include "kalman_filter.h"

static const float PROCESS_NOISE  = 0.01f;
static const float MEASURE_NOISE  = 0.1f;

static KalmanSensor kFL, kCL, kCR, kFR;

void kalman_init(KalmanSensor* s) {
  s->estimate = 0.0f;
  s->errorCov = 1.0f;
}

float kalman_update(KalmanSensor* s, float measurement) {
  s->errorCov = s->errorCov + PROCESS_NOISE;
  float gain  = s->errorCov / (s->errorCov + MEASURE_NOISE);
  s->estimate = s->estimate + gain * (measurement - s->estimate);
  s->errorCov = (1.0f - gain) * s->errorCov;
  return s->estimate;
}

void kalman_filter_init() {
  kalman_init(&kFL);
  kalman_init(&kCL);
  kalman_init(&kCR);
  kalman_init(&kFR);
}

FilteredIR kalman_filter_update(bool fl, bool cl, bool cr, bool fr) {
  FilteredIR result;
  result.farLeft     = kalman_update(&kFL, fl ? 1.0f : 0.0f);
  result.centerLeft  = kalman_update(&kCL, cl ? 1.0f : 0.0f);
  result.centerRight = kalman_update(&kCR, cr ? 1.0f : 0.0f);
  result.farRight    = kalman_update(&kFR, fr ? 1.0f : 0.0f);
  return result;
}
