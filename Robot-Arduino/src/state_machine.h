#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <Arduino.h>

// All possible robot operating states
typedef enum {
    STATE_IDLE,            // Waiting for a mission (RF trigger in Stage 3)
    STATE_LINE_FOLLOW,     // PID line following toward the next station
    STATE_STATION_DETECT,  // All-4-IR black confirmed — verifying it is the target station
    STATE_AT_STATION,      // Arrived at target station — ready to open box
    STATE_WAITING_BOX,     // Waiting for operator to open and close the box door
    STATE_BOX_TIMEOUT,     // 3-minute timer elapsed without box interaction — skipping
    STATE_OBSTACLE,        // Obstacle blocking path — halted, waiting for clearance
    STATE_RETURN_HOME,     // All deliveries done — navigating back to station 0
    STATE_MANUAL,          // RF joystick control mode (behavior wired in Stage 3)
    STATE_ESTOP            // Emergency stop — all motion halted until external resume
} RobotState;

// Current operating state — readable by RF module in Stage 3
extern RobotState currentState;

// UART bridge station confirmation status (waiting / confirmed / ignored)
extern uint8_t stationConfirmStatus;

// Initialize state machine and reset all internal variables
void sm_init();

// Main dispatcher — call every loop() iteration
void sm_run();

// Perform a state transition: update state, reset timer, update LCD
void sm_transition(RobotState newState);

// Set the target station variable — used by RF module after loading a new mission
void sm_set_target(uint8_t station);

// Set the cross-line station counter — used by RF module to re-sync position
void sm_set_station_counter(uint8_t val);

// Startup check: confirm robot is physically at station 0 before entering IDLE
void sm_confirm_home_position();

#endif
