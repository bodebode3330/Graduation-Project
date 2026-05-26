#include "buzzer.h"

// Configure buzzer pin as output and ensure it starts silent
void buzzer_init() {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
}

// Block for duration_ms with buzzer ON, then turn it OFF
void buzzer_beep(uint16_t duration_ms) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration_ms);
    digitalWrite(BUZZER_PIN, LOW);
}

// Repeat an ON/OFF beep cycle 'times' times
void buzzer_beep_pattern(uint8_t times, uint16_t on_ms, uint16_t off_ms) {
    for (uint8_t i = 0; i < times; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(on_ms);
        digitalWrite(BUZZER_PIN, LOW);
        // Skip trailing OFF delay on the final cycle to avoid unnecessary wait
        if (i < times - 1) {
            delay(off_ms);
        }
    }
}

// Three short fast beeps — warns operator before a station is skipped on timeout
void buzzer_warning() {
    buzzer_beep_pattern(3, 80, 80);
}

// Non-blocking alternating tone for continuous obstacle alert — call repeatedly in loop()
void buzzer_obstacle() {
    // Toggle buzzer every 300 ms without using delay()
    const uint16_t TOGGLE_INTERVAL_MS = 300;
    static bool          buzzerState   = false;
    static unsigned long lastToggle    = 0;

    unsigned long now = millis();
    if (now - lastToggle >= TOGGLE_INTERVAL_MS) {
        buzzerState = !buzzerState;
        digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
        lastToggle = now;
    }
}
