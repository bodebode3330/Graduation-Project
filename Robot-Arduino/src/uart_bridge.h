#ifndef UART_BRIDGE_H
#define UART_BRIDGE_H

#include <Arduino.h>
#include "config.h"

// Initialize hardware Serial at ESP32_BAUD (D0=RX from ESP32, D1=TX to ESP32)
void uart_bridge_init();

// Non-blocking: send IR readings to ESP32 every 50ms, parse incoming commands
void uart_bridge_update();

// Send "STN:DETECT" to request station confirmation from ESP32
void uart_bridge_request_confirm();

// ✨ NEW: Get the latest confidence level (0.0 - 1.0) from ESP32
float uart_bridge_get_confidence();

#endif
