#include "motors.h"

// Initialize all motor control pins as outputs and stop motors
void motors_init() {
    pinMode(LEFT_RPWM,  OUTPUT);
    pinMode(LEFT_LPWM,  OUTPUT);
    pinMode(RIGHT_RPWM, OUTPUT);
    pinMode(RIGHT_LPWM, OUTPUT);
    motors_stop();
}

// Brake all motors by zeroing all PWM outputs
void motors_stop() {
    analogWrite(LEFT_RPWM,  0);
    analogWrite(LEFT_LPWM,  0);
    analogWrite(RIGHT_RPWM, 0);
    analogWrite(RIGHT_LPWM, 0);
}

// Set left and right motor speeds independently; range -255 to +255
void motors_set(int leftSpeed, int rightSpeed) {
    // Clamp inputs to valid PWM range
    leftSpeed  = constrain(leftSpeed,  -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    // Apply left motor direction and magnitude
    if (leftSpeed > 0) {
        analogWrite(LEFT_RPWM, leftSpeed);
        analogWrite(LEFT_LPWM, 0);
    } else if (leftSpeed < 0) {
        analogWrite(LEFT_RPWM, 0);
        analogWrite(LEFT_LPWM, -leftSpeed);
    } else {
        analogWrite(LEFT_RPWM, 0);
        analogWrite(LEFT_LPWM, 0);
    }

    // Apply right motor direction and magnitude
    if (rightSpeed > 0) {
        analogWrite(RIGHT_RPWM, rightSpeed);
        analogWrite(RIGHT_LPWM, 0);
    } else if (rightSpeed < 0) {
        analogWrite(RIGHT_RPWM, 0);
        analogWrite(RIGHT_LPWM, -rightSpeed);
    } else {
        analogWrite(RIGHT_RPWM, 0);
        analogWrite(RIGHT_LPWM, 0);
    }
}

// Drive both sides forward at the given speed
void motors_forward(uint8_t speed) {
    motors_set(speed, speed);
}

// Drive both sides backward at the given speed
void motors_backward(uint8_t speed) {
    motors_set(-speed, -speed);
}

// Pivot left: left side backward, right side forward
void motors_turn_left(uint8_t speed) {
    motors_set(-speed, speed);
}

// Pivot right: left side forward, right side backward
void motors_turn_right(uint8_t speed) {
    motors_set(speed, -speed);
}

// Spin in place to the left (same as turn_left — alias for semantic clarity)
void motors_spin_left(uint8_t speed) {
    motors_set(-speed, speed);
}

// Spin in place to the right (same as turn_right — alias for semantic clarity)
void motors_spin_right(uint8_t speed) {
    motors_set(speed, -speed);
}
