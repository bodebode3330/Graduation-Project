#pragma once

struct KalmanSensor {
  float estimate;
  float errorCov;
};

struct FilteredIR {
  float farLeft;
  float centerLeft;
  float centerRight;
  float farRight;
};

void kalman_init(KalmanSensor* s);
float kalman_update(KalmanSensor* s, float measurement);

void kalman_filter_init();
FilteredIR kalman_filter_update(bool fl, bool cl, bool cr, bool fr);
