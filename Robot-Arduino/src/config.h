#ifndef CONFIG_H
#define CONFIG_H

// ─── Mission parameters ───────────────────────────────────────────────────────

#define TOTAL_STATIONS        5          // Total number of delivery stations (not counting home)
#define HOME_STATION          0          // Station number for the home/base position

// ─── Timing constants ─────────────────────────────────────────────────────────

#define BOX_TIMEOUT_MS        180000UL  // 3 minutes — skip station if box not interacted
#define BOX_WARNING_MS        150000UL  // 2.5 minutes — warn operator 30s before timeout
#define OBSTACLE_RECHECK_MS   500        // Poll ultrasonic every 500ms when halted
#define STATION_PAUSE_MS      300        // Brief stop pause before opening box
#define CROSS_LINE_CONFIRM_MS 80         // All-4-black must persist 80ms to confirm station
#define APPROACH_SPEED        45         // Reduced speed when centre sensors detect a line

// ─── ESP32 UART bridge ────────────────────────────────────────────────────────
// Hardware Serial (D0/D1) is used exclusively — no SoftwareSerial

#define ESP32_BAUD       9600

// Station detection confirmation modes
#define STN_WAITING      0    // No station event pending
#define STN_PENDING      1    // Arduino detected cross-line, waiting for ESP32
#define STN_CONFIRMED    2    // ESP32 confirmed — count this station
#define STN_IGNORED      3    // ESP32 rejected — ignore this cross-line

#endif
