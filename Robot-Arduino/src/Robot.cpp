/*
 * robot.ino — Stage 3: RF Receiver & Remote Control Integration
 *
 * Autonomous cargo delivery robot — Arduino Uno (ATmega328P)
 *
 * Stage 1: Hardware abstraction layer (motors, IR/PID, ultrasonic, buzzer,
 *          limit switch, LCD) — motors.h, ir_sensors.h, ultrasonic.h,
 *          buzzer.h, limit_switch.h, lcd_display.h
 * Stage 2: Mission queue management and full state machine —
 *          mission.h, state_machine.h
 * Stage 3: 433MHz RF receiver (RH_ASK), packet parsing, joystick manual
 *          control, and ESTOP — rf_receiver.h
 * Stage 4: ESP32 UART bridge for IR telemetry and remote PID tuning —
 *          uart_bridge.h
 *
 * Pin summary
 *   IR sensors      : 2, A1, 4, A3  (FAR_LEFT=2, CENTER_LEFT=A1, CENTER_RIGHT=4, FAR_RIGHT=7)
 *   Left  motor PWM : 5 (RPWM), 6 (LPWM)
 *   Right motor PWM : 3 (RPWM), 11 (LPWM)  — pins 9/10 freed; RH_ASK/Timer1 conflict
 *   Ultrasonic      : TRIG=8, ECHO=A2       — pin 11 freed; moved to A2
 *   Buzzer          : 12
 *   Limit switch    : 13
 *   LCD I2C         : A4 (SDA), A5 (SCL)
 *   RF receiver     : A0 (RH_ASK / 433MHz ASK module)
 *   ESP32 UART      : D0 (RX from ESP32), D1 (TX to ESP32)
 */

#include <RH_ASK.h>
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Stage 1 — hardware abstraction layer
#include "config.h"
#include "motors.h"
#include "ir_sensors.h"
#include "ultrasonic.h"
#include "buzzer.h"
#include "limit_switch.h"
#include "lcd_display.h"

// Stage 2 — mission logic and state machine
#include "mission.h"
#include "state_machine.h"

// Stage 3 — RF receiver and remote control
#include "rf_receiver.h"

// Stage 4 — ESP32 UART bridge
#include "uart_bridge.h"

void ir_sensors_init();

// ─── setup ───────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(9600);

    // Stage 1: hardware modules
    motors_init();          // Motor driver pins + brake
    pid_init();             // Reset PID state variables
    ultrasonic_init();      // HC-SR04 TRIG / ECHO
    buzzer_init();          // Active buzzer pin
    limit_switch_init();    // Box door limit switch with pull-up

    // ir_sensors_init sets IR pin modes (defined below in this file)
    ir_sensors_init();

    // LCD splash screen ("Delivery Robot / Initializing...") — blocks 1500 ms
    lcd_init();

    // Stage 2: mission queue and state machine
    mission_init();         // Zero mission data, place robot at HOME_STATION
    sm_init();              // Enter STATE_IDLE, reset all SM variables

    // Stage 3: RF receiver — must be last init so all state is ready
    rf_receiver_init();     // Initialise RH_ASK driver, zero packet struct

    // Stage 4: ESP32 UART bridge
    uart_bridge_init();     // Initialise hardware Serial (D0=RX, D1=TX) for ESP32

    // Startup position check — confirms robot is at station 0 before entering IDLE
    sm_confirm_home_position();
}

// ─── loop ────────────────────────────────────────────────────────────────────

void loop() {
    rf_receiver_update();   // Poll RF; applies any valid packet to state machine
    uart_bridge_update();   // Send IR telemetry; parse incoming ESP32 commands
    sm_run();               // Advance state machine one iteration
}

// ─── ir_sensors_init (pin setup helper — defined here, called in setup) ──────
// IR pins default to INPUT on the Uno but are set explicitly for clarity.

void ir_sensors_init() {
    pinMode(IR_FAR_LEFT,     INPUT);
    pinMode(IR_CENTER_LEFT,  INPUT);
    pinMode(IR_CENTER_RIGHT, INPUT);
    pinMode(IR_FAR_RIGHT,    INPUT);
}
