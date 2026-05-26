#pragma once
#include <Arduino.h>

void adaptive_pid_init(float initialKP, float initialKD, uint8_t initialSpeed);
void adaptive_pid_add_error(float error);
void adaptive_pid_update(HardwareSerial& port);
// ✨ NEW: Update PID values with every frame (every 50ms) for continuous adaptation
void adaptive_pid_update_per_frame(HardwareSerial& port);
// ✨ NEW: Get current PID values for debugging
void adaptive_pid_get_values(float* outKP, float* outKD, uint8_t* outSpeed);
// ✨ NEW: Get suggested speed correction based on error trend
int8_t adaptive_pid_get_speed_correction();
