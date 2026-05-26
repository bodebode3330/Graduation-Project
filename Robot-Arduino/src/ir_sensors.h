#ifndef IR_SENSORS_H
#define IR_SENSORS_H

#include <Arduino.h>

// IR sensor digital input pins (HIGH = black line detected)
// IR_CENTER_LEFT on A1: pin 3 is used by RIGHT_RPWM (Timer1 conflict workaround)
// IR_FAR_RIGHT   on A3: pin 7 reassigned to free a digital pin
#define IR_FAR_LEFT     2
#define IR_CENTER_LEFT  A1
#define IR_CENTER_RIGHT 4
#define IR_FAR_RIGHT    A3

// PID controller tuning constants
#define PID_KP      30.0f
#define PID_KI       0.0f
#define PID_KD      220.0f
#define BASE_SPEED  85     // Base forward speed (0-255)
#define MAX_SPEED   200     // Maximum motor speed cap

// Struct holding a snapshot of all four IR sensor readings
struct IRReadings {
    bool farLeft;
    bool centerLeft;
    bool centerRight;
    bool farRight;
};

// Read all four IR sensors and return their current state
IRReadings ir_read();

// Returns true if all four sensors detect black (cross-line intersection)
bool ir_all_black(IRReadings r);

// Returns true when the robot is over a strong cross-line signature
// (either full intersection or centered line with at least one outer sensor).
bool ir_cross_line(IRReadings r);

// Returns true if the two middle sensors detect black (station detection)
bool ir_middle_black(IRReadings r);

// Returns true if all four sensors detect white (line lost)
bool ir_all_white(IRReadings r);

// Compute weighted error for PID: weights are -3, -1, +1, +3 per sensor
int ir_compute_error(IRReadings r);

// Reset PID internal state (call before starting line-following)
void pid_init();

// Update PID tuning values from external control
void pid_set_kp(float kp);
void pid_set_kd(float kd);
void pid_set_base_speed(uint8_t speed);

// Read sensors, compute PID output, and drive motors accordingly
// Pass reverse=true to invert steering correction when travelling back toward home
void pid_compute_and_drive(bool reverse);

#endif
