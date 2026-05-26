#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

// BTS7960 Driver 1 — Left side motors
#define LEFT_RPWM  5   // PWM: forward
#define LEFT_LPWM  6   // PWM: reverse

// BTS7960 Driver 2 — Right side motors
// Pins 9 and 10 avoided: RH_ASK uses Timer1 which disables PWM on those pins
#define RIGHT_RPWM 3   // PWM: forward
#define RIGHT_LPWM 11  // PWM: reverse

// Initialize motor pins and stop all motors
void motors_init();

// Brake all motors immediately
void motors_stop();

// Set left and right speeds independently (-255 to +255)
void motors_set(int leftSpeed, int rightSpeed);

// Drive both sides forward at given speed (0-255)
void motors_forward(uint8_t speed);

// Drive both sides backward at given speed (0-255)
void motors_backward(uint8_t speed);

// Pivot left: left backward, right forward
void motors_turn_left(uint8_t speed);

// Pivot right: left forward, right backward
void motors_turn_right(uint8_t speed);

// Spin in place to the left (both sides driven oppositely)
void motors_spin_left(uint8_t speed);

// Spin in place to the right (both sides driven oppositely)
void motors_spin_right(uint8_t speed);

#endif
