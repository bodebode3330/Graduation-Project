#include "uart_bridge.h"
#include "ir_sensors.h"
#include "state_machine.h"
#include "ultrasonic.h"

// Timestamp of last IR data transmission to ESP32
static uint32_t lastIRSendTime = 0;
#define IR_SEND_INTERVAL_MS  50   // Send IR snapshot every 50ms

// ✨ NEW: Store latest confidence level from ESP32 (0.0 - 1.0)
static float lastConfidence = 0.0f;

// Forward declaration — defined below, called from uart_bridge_update()
static void uart_bridge_parse(const char* cmd);

void uart_bridge_init() {
    Serial.begin(ESP32_BAUD);
}

void uart_bridge_update() {
    // ── Send IR readings to ESP32 every 50ms ──────────────────────────────────
    if (millis() - lastIRSendTime >= IR_SEND_INTERVAL_MS) {
        lastIRSendTime = millis();

        IRReadings r = ir_read();
        // Format: "IR:XXXX\n" where each X is 0 or 1 (FL CL CR FR)
        Serial.print(F("IR:"));
        Serial.print(r.farLeft     ? '1' : '0');
        Serial.print(r.centerLeft  ? '1' : '0');
        Serial.print(r.centerRight ? '1' : '0');
        Serial.println(r.farRight  ? '1' : '0');

        // Also send ultrasonic distance
        float dist = ultrasonic_read_cm();
        Serial.print(F("DST:"));
        Serial.println(dist, 1);
    }

    // ── Parse incoming commands from ESP32 ────────────────────────────────────
    // Commands arrive as newline-terminated strings: "CMD:VALUE\n"
    // Read one complete line per call (non-blocking)
    while (Serial.available()) {
        static char    rxBuf[24];
        static uint8_t rxIdx = 0;

        char c = Serial.read();

        if (c == '\n') {
            rxBuf[rxIdx] = '\0'; // Null-terminate
            uart_bridge_parse(rxBuf);
            rxIdx = 0;           // Reset buffer for next line
        } else if (rxIdx < sizeof(rxBuf) - 1) {
            rxBuf[rxIdx++] = c;
        }
    }
}

// Parse a single null-terminated command string from ESP32
// Supported commands:
//   STN:CONFIRM  — station detection confirmed
//   STN:IGNORE   — station detection rejected (noise)
//   PID:KP:KD    — update PID gains (e.g. "PID:28.5:195.0")
//   SPD:VALUE    — update BASE_SPEED (e.g. "SPD:80")
//   CONF:VALUE   — station confidence level (e.g. "CONF:0.78")
static void uart_bridge_parse(const char* cmd) {
    if (strncmp(cmd, "STN:CONFIRM", 11) == 0) {
        stationConfirmStatus = STN_CONFIRMED;

    } else if (strncmp(cmd, "STN:IGNORE", 10) == 0) {
        stationConfirmStatus = STN_IGNORED;

    } else if (strncmp(cmd, "PID:", 4) == 0) {
        float newKP = 0, newKD = 0;
        if (sscanf(cmd + 4, "%f:%f", &newKP, &newKD) == 2) {
            pid_set_kp(newKP);
            pid_set_kd(newKD);
        }

    } else if (strncmp(cmd, "SPD:", 4) == 0) {
        uint8_t newSpd = (uint8_t)atoi(cmd + 4);
        pid_set_base_speed(newSpd);

    } else if (strncmp(cmd, "CONF:", 5) == 0) {
        // ✨ NEW: Store confidence level from ESP32 (0.0 - 1.0)
        lastConfidence = atof(cmd + 5);
    }
}

void uart_bridge_request_confirm() {
    Serial.println(F("STN:DETECT"));
}

// ✨ NEW: Get latest confidence level from ESP32
float uart_bridge_get_confidence() {
    return lastConfidence;
}
