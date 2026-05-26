#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

// HC-SR04 ultrasonic sensor pins
// ULTRASONIC_ECHO moved to A2: pin 11 is now used by RIGHT_LPWM (Timer1 conflict workaround)
#define ULTRASONIC_TRIG 8
#define ULTRASONIC_ECHO A2

// Distance in cm below which an obstacle is reported
#define OBSTACLE_THRESHOLD_CM 20

// Initialize TRIG as output and ECHO as input
void ultrasonic_init();

// Trigger a measurement and return distance in centimetres; 999.0 on timeout
float ultrasonic_read_cm();

// Returns true if the measured distance is below the obstacle threshold
bool ultrasonic_obstacle_detected();

#endif
