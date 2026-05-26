#pragma once
#include <Arduino.h>
#include "kalman_filter.h"

void station_detector_init();
void station_detector_add_sample(FilteredIR sample);
// Evaluate collected IR history and send STN:CONFIRM or STN:IGNORE on `port`.
// Returns true if a station was confirmed (CONFIRM sent), false otherwise.
bool station_detector_evaluate(HardwareSerial& port);
// Get current confidence level (0.0 - 1.0) for ongoing station detection
float station_detector_get_confidence();
// Check if confidence is progressively improving (pattern stabilizing)
bool station_detector_is_improving();
