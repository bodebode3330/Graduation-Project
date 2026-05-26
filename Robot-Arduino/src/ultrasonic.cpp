#include "ultrasonic.h"

// Configure HC-SR04 pins: TRIG as output, ECHO as input
void ultrasonic_init() {
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    digitalWrite(ULTRASONIC_TRIG, LOW); // Ensure TRIG starts LOW
}

// Trigger a distance measurement and return result in centimetres
float ultrasonic_read_cm() {
    // Send a clean 10 µs HIGH pulse on TRIG
    digitalWrite(ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);

    // Read the echo pulse duration with a 25 ms timeout (prevents indefinite block)
    unsigned long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 25000UL);

    // pulseIn() returns 0 on timeout — treat as out-of-range
    if (duration == 0) return 999.0f;

    // Convert pulse duration to centimetres: speed of sound ≈ 0.034 cm/µs, round trip /2
    return (float)duration * 0.034f / 2.0f;
}

// Returns true if a nearby obstacle is within the detection threshold
bool ultrasonic_obstacle_detected() {
    return ultrasonic_read_cm() < (float)OBSTACLE_THRESHOLD_CM;
}
