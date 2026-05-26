#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>

// Active buzzer digital output pin
#define BUZZER_PIN 12

// Configure buzzer pin as output and ensure it starts silent
void buzzer_init();

// Block for duration_ms with buzzer ON, then leave buzzer OFF
void buzzer_beep(uint16_t duration_ms);

// Repeat ON/OFF cycle 'times' times with configurable durations
void buzzer_beep_pattern(uint8_t times, uint16_t on_ms, uint16_t off_ms);

// Three short fast beeps — timeout warning before skipping a station
void buzzer_warning();

// Non-blocking alternating tone used while obstacle is present (call in loop)
void buzzer_obstacle();

#endif
