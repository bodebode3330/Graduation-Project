/*
 * ------------------------------------------------------------
 *  File:    config.h
 *  Purpose: Central configuration file for the delivery remote firmware.
 *
 *  This file defines every pin number, timing constant, and hardware
 *  parameter used across the project. All other files include this header
 *  so that changing a value here automatically updates every module
 *  that depends on it — no need to hunt through multiple files.
 *
 *  Dependencies: none (only uses preprocessor #define directives)
 * ------------------------------------------------------------
 */

#ifndef CONFIG_H
#define CONFIG_H

/* ── Timing constants ───────────────────────────────────────────────────────
 *
 * DEBOUNCE_MS: How long (in milliseconds) a button must hold its new state
 *   before we consider the press valid. Mechanical buttons physically bounce
 *   (rapidly switch on/off) for a few milliseconds when pressed. 50ms is long
 *   enough to outlast the bounce but short enough that the operator never
 *   notices the delay.
 *
 * RF_SEND_INTERVAL_MS: How often (in milliseconds) the remote re-transmits
 *   the current packet. 100ms (10 packets/second) keeps the robot responsive
 *   while leaving enough air time between transmissions to avoid self-jamming.
 */
#define DEBOUNCE_MS         50
#define RF_SEND_INTERVAL_MS 100

/* ── Reset constants ────────────────────────────────────────────────────────
 *
 * RF_MODE_RESET: The mode byte sent in a reset packet. Value 0x04 follows the
 *   existing sequence (AUTO=0x01, MANUAL=0x02, ESTOP=0x03) and must be handled
 *   on the receiver side to trigger a full robot state reset.
 *
 * RESET_HOLD_MS: How long (in milliseconds) the ESTOP button must be held
 *   continuously to trigger a reset. 3000ms (3 seconds) is long enough to
 *   prevent accidental resets while still being reachable in an emergency.
 */
#define RF_MODE_RESET       0x04
#define RESET_HOLD_MS       3000

/* ── Joystick tuning ────────────────────────────────────────────────────────
 *
 * JOY_DEADZONE: If the raw analog reading is within this many counts of center,
 *   we treat the stick as centered. This prevents the robot from slowly drifting
 *   when the operator is not touching the joystick — analog sticks rarely rest
 *   at exactly 512 due to mechanical tolerance.
 *   ±40 counts out of 1023 is roughly ±4% of full range, a comfortable threshold.
 *
 * JOY_CENTER: The expected resting value of the joystick analog output.
 *   The Arduino ADC produces 0-1023 (10-bit). An ideal centered joystick
 *   reads 511.5, which rounds to 512.
 */
#define JOY_DEADZONE        40
#define JOY_CENTER          512

/* ── Room count ─────────────────────────────────────────────────────────────
 *
 * TOTAL_ROOMS: The number of delivery destinations this remote supports.
 *   Used in loops that iterate over room buttons and in display formatting.
 *   Changing this value here automatically adjusts all loops — no other
 *   files need editing if rooms are added or removed.
 */
#define TOTAL_ROOMS         5

/* ── Room button pin assignments (digital pins, INPUT_PULLUP) ───────────────
 *
 * Each room has a dedicated momentary push button wired between the pin
 * and GND. INPUT_PULLUP means the pin reads HIGH when the button is open
 * and LOW when the button is pressed — no external resistor is needed.
 * Pins 2-6 are used so they don't conflict with Serial (0,1) or SPI (10-13).
 */
#define BTN_ROOM_1    2   /* Digital pin 2 — Room 1 selection button */
#define BTN_ROOM_2    3   /* Digital pin 3 — Room 2 selection button */
#define BTN_ROOM_3    4   /* Digital pin 4 — Room 3 selection button */
#define BTN_ROOM_4    5   /* Digital pin 5 — Room 4 selection button */
#define BTN_ROOM_5    6   /* Digital pin 6 — Room 5 selection button */

/* ── Control button pin assignments (digital pins, INPUT_PULLUP) ────────────
 *
 * BTN_SEND:  Pressing this dispatches the current room queue to the robot.
 *            Ignored if no rooms are selected.
 *
 * BTN_MODE:  Toggles between AUTO and MANUAL mode. If ESTOP is active,
 *            pressing MODE resumes normal operation.
 *
 * BTN_ESTOP: Emergency stop — immediately halts the robot regardless of mode.
 *            This button is checked first every loop so it can never be blocked.
 */
#define BTN_SEND      7   /* Digital pin 7 — Send mission button */
#define BTN_MODE      8   /* Digital pin 8 — AUTO / MANUAL toggle button */
#define BTN_ESTOP     9   /* Digital pin 9 — Emergency Stop button */

/* ── Joystick analog pin assignments ────────────────────────────────────────
 *
 * Analog pins do not require pinMode() — they are always inputs.
 * The joystick module provides two potentiometer axes, each outputting
 * a voltage that the ADC converts to a 0-1023 integer.
 *
 * JOY_X_PIN: Left/right axis. Left ≈ 0, center ≈ 512, right ≈ 1023.
 * JOY_Y_PIN: Forward/backward axis. Back ≈ 0, center ≈ 512, forward ≈ 1023.
 */
#define JOY_X_PIN     A0  /* Analog pin A0 — Joystick horizontal (X) axis */
#define JOY_Y_PIN     A1  /* Analog pin A1 — Joystick vertical   (Y) axis */

/* ── RF transmitter pin assignment ──────────────────────────────────────────
 *
 * RF_TX_PIN: The data output pin connected to the DATA line of the 433MHz
 *   transmitter module. Pin 12 is chosen because it does not conflict with
 *   SPI (SS=10, MOSI=11, MISO=12, SCK=13) in transmit-only mode — RadioHead
 *   bit-bangs its own timing and does not use the hardware SPI peripheral,
 *   so MISO (pin 12) is free for this purpose when RX is disabled (pin 255).
 */
#define RF_TX_PIN     12  /* Digital pin 12 — RF 433MHz transmitter data pin */

#endif
