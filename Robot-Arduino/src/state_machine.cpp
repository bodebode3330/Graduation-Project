#include "state_machine.h"
#include "config.h"
#include "mission.h"
#include "motors.h"
#include "ir_sensors.h"
#include "ultrasonic.h"
#include "buzzer.h"
#include "limit_switch.h"
#include "lcd_display.h"
#include "uart_bridge.h"

// ─── Public state ─────────────────────────────────────────────────────────────

RobotState currentState = STATE_IDLE;

// ─── Private state machine variables ─────────────────────────────────────────

static uint32_t    stateTimer         = 0;   // millis() when current state was entered
static uint32_t    warningTimer       = 0;   // tracks when box-timeout warning was sent
static bool        warningGiven       = false; // true once the 2.5-min beep has fired
static uint8_t     stationCounter     = 0;   // cross-line intersections counted since last known station
static uint8_t     targetStation      = HOME_STATION; // station the robot is currently heading for
uint8_t            stationConfirmStatus = STN_WAITING; // UART bridge station confirmation state
static bool        ignoreCurrentIntersection = false; // true while skipping a false positive line
static bool        crossLineConfirmed = false; // debounce flag — true while 4-black pulse is timing
static uint32_t    crossLineTimer     = 0;   // millis() when all-4-black was first observed
static RobotState  savedState         = STATE_LINE_FOLLOW; // state to resume after obstacle clears
static uint8_t     stationsToSkip     = 0;    // intermediate stations to pass before reaching home
static uint8_t     returnHomeCounter  = 0;    // cross-lines counted while returning home
static bool        leavingStartCrossLine = false; // true while driving off the starting cross-line

// ✨ NEW: Dynamically calculated cross-line confirmation time based on confidence
static uint32_t    dynamicConfirmMs   = CROSS_LINE_CONFIRM_MS;

// ✨ NEW: Calculate optimal confirmation time based on ESP32 confidence
static uint32_t sm_get_dynamic_confirm_ms() {
    float conf = uart_bridge_get_confidence();
    
    if (conf > 0.85f) {
        return 50;    // Very high confidence — speed up to 50ms
    } else if (conf > 0.70f) {
        return CROSS_LINE_CONFIRM_MS;  // Normal confidence — use default 80ms
    } else if (conf > 0.55f) {
        return 100;   // Lower confidence — increase to 100ms
    } else {
        return 120;   // Very low confidence — use max 120ms
    }
}

// ─── LCD helper: update display for a given state ────────────────────────────

static void sm_update_lcd(RobotState state) {
    switch (state) {
        case STATE_IDLE:
            lcd_show_message("IDLE", "Waiting mission");
            break;
        case STATE_LINE_FOLLOW:
            lcd_show_status(mission.currentStation, targetStation, "AUTO");
            break;
        case STATE_STATION_DETECT:
            lcd_show_status(mission.currentStation, targetStation, "DET");
            break;
        case STATE_AT_STATION:
            lcd_show_waiting_box(targetStation, BOX_TIMEOUT_MS / 1000);
            break;
        case STATE_WAITING_BOX:
            lcd_show_waiting_box(mission.currentStation, BOX_TIMEOUT_MS / 1000);
            break;
        case STATE_BOX_TIMEOUT:
            lcd_show_message("Timeout!", "Skipping Room");
            break;
        case STATE_OBSTACLE:
            lcd_show_obstacle();
            break;
        case STATE_RETURN_HOME:
            lcd_show_home();
            break;
        case STATE_MANUAL:
            lcd_show_message("MANUAL MODE", "Joystick Active");
            break;
        case STATE_ESTOP:
            lcd_show_estop();
            break;
    }
}

// ─── Transition ───────────────────────────────────────────────────────────────

// Perform a state transition: save previous context, update LCD, reset timer
void sm_transition(RobotState newState) {
    // Save previous state so obstacle handler can resume it on clearance
    if (newState == STATE_OBSTACLE) {
        savedState = currentState;
    }

    // Reset box-warning flag whenever (re-)entering the waiting state
    if (newState == STATE_WAITING_BOX) {
        warningGiven = false;
    }

    // When entering LINE_FOLLOW, check if the robot is already on a cross-line.
    // If it is, set the flag so handle_line_follow() drives off it before counting.
    if (newState == STATE_LINE_FOLLOW) {
        IRReadings r = ir_read();
        leavingStartCrossLine = ir_cross_line(r); // true if starting on a station/cross-line
        crossLineConfirmed    = false;
        crossLineTimer        = 0;
    }

    if (newState == STATE_STATION_DETECT) {
        stationConfirmStatus = STN_PENDING;
        uart_bridge_request_confirm();
    }

    // Return-home setup — calculate how many stations to pass before home
    if (newState == STATE_RETURN_HOME) {
        // Special case: already at home — skip the state entirely
        if (mission.currentStation == HOME_STATION) {
            mission_clear();
            buzzer_beep_pattern(4, 200, 100);
            lcd_show_message("Home!", "Mission Done");
            currentState = STATE_IDLE;
            stateTimer   = millis();
            sm_update_lcd(STATE_IDLE);
            return;
        }
        // Track how many intermediate cross-lines to pass before station 0
        stationsToSkip    = TOTAL_STATIONS - mission.currentStation;
        returnHomeCounter = 0;
        crossLineConfirmed = false;
        crossLineTimer     = 0;
    }

    currentState = newState;
    stateTimer   = millis();

    sm_update_lcd(newState);
}

// ─── State handlers (static — only called from sm_run) ───────────────────────

// STATE_IDLE — hold position, wait for missionActive to be set
static void handle_idle() {
    motors_stop();
    // Transition to line-following as soon as a mission is queued
    if (mission.missionActive) {
        sm_transition(STATE_LINE_FOLLOW);
    }
}

// STATE_LINE_FOLLOW — PID line following with station detection and obstacle check
static void handle_line_follow() {
    // ── Startup clearance: drive off the cross-line we started on ────────────
    // Fires once per LINE_FOLLOW entry when the robot begins on a cross-line.
    // Prevents the starting position from being counted as an intersection.
    if (leavingStartCrossLine) {
        if (ultrasonic_obstacle_detected()) {
            motors_stop();
            sm_transition(STATE_OBSTACLE);
            return;
        }
        IRReadings rc = ir_read();
        if (ir_cross_line(rc)) {
            // Still on the starting cross-line — keep driving forward
            motors_forward(BASE_SPEED);
        } else {
            // Cleared the starting cross-line — normal operation can begin
            leavingStartCrossLine = false;
            crossLineConfirmed    = false;
            crossLineTimer        = 0;
        }
        return; // skip all PID and detection while clearing
    }

    // Non-blocking crossing sub-state — used when passing a non-target station
    enum CrossPhase { CP_IDLE, CP_WAIT_CLEAR, CP_EXTRA_PUSH };
    static CrossPhase crossPhase      = CP_IDLE;
    static uint32_t   crossPhaseTimer = 0;
    static uint32_t   crossLineLostTimer = 0; // allow short gaps in line detection

    // Obstacle check has priority — halt immediately if path is blocked
    if (ultrasonic_obstacle_detected()) {
        motors_stop();
        sm_transition(STATE_OBSTACLE);
        return;
    }

    IRReadings r = ir_read();

    // Ignore a false-positive intersection until the line is fully cleared.
    if (ignoreCurrentIntersection) {
        if (ir_cross_line(r)) {
            motors_forward(BASE_SPEED);
            return;
        }
        ignoreCurrentIntersection = false;
        crossLineConfirmed      = false;
        crossLineTimer          = 0;
    }

    // While crossing a non-target station, skip PID and station detection
    if (crossPhase != CP_IDLE) {
        if (crossPhase == CP_WAIT_CLEAR) {
            // Drive forward until the robot has fully cleared the intersection line
            motors_forward(BASE_SPEED);
            IRReadings r2 = ir_read();
            if (!ir_cross_line(r2)) {
                crossPhase      = CP_EXTRA_PUSH;
                crossPhaseTimer = millis();
            }
        } else { // CP_EXTRA_PUSH
            // Push an extra 150 ms to clear the line completely before resuming PID
            motors_forward(BASE_SPEED);
            if (millis() - crossPhaseTimer >= 150UL) {
                crossPhase         = CP_IDLE;
                // Bug 2 fix: clear debounce flags so the next intersection is not
                // double-counted on the first loop iteration after crossing finishes
                crossLineConfirmed = false;
                crossLineTimer     = 0;
            }
        }
        return;
    }

    // Relaxed station detection: accept a strong cross-line signature rather
    // than requiring all four sensors to see black at exactly the same time.
    if (ir_cross_line(r)) {
        // Bug 3 fix: motors_stop() only while the debounce window is still open;
        // it must not fire after a transition or after CP_WAIT_CLEAR is started
        crossLineLostTimer = 0;
        if (!crossLineConfirmed) {
            // Start timing the confirmation window and hold position
            crossLineConfirmed = true;
            crossLineTimer     = millis();
            dynamicConfirmMs   = sm_get_dynamic_confirm_ms();  // ✨ Update dynamic timing
            motors_stop();
        } else if (millis() - crossLineTimer >= dynamicConfirmMs) {
            // Confirmed real station line — count it
            stationCounter++;
            crossLineConfirmed = false;
            crossLineTimer     = 0;

            if (stationCounter == targetStation) {
                // This is our target station; new state controls motors from here
                sm_transition(STATE_STATION_DETECT);
            } else {
                // Non-blocking crossing — crossing push starts next iteration
                crossPhase      = CP_WAIT_CLEAR;
                crossPhaseTimer = millis();
            }
            // no motors_stop() here — let the crossing sub-state or new state control motors
        } else {
            motors_stop(); // still waiting for debounce window to expire
        }
    } else {
        // Allow a short interruption in the line signal before giving up detection.
        if (crossLineConfirmed) {
            if (crossLineLostTimer == 0) crossLineLostTimer = millis();
            if (millis() - crossLineLostTimer < dynamicConfirmMs) {
                motors_stop();
                return;
            }
        }

        // Not at an intersection — reset debounce flag and follow line
        crossLineConfirmed = false;
        crossLineTimer     = 0;
        crossLineLostTimer = 0;

        // Approach detection: centre sensors hit the cross-line before the outer sensors.
        // Slow to APPROACH_SPEED so the full cross-line persists long enough to confirm.
        IRReadings ra = ir_read();
        if (ir_middle_black(ra) && !ir_all_black(ra)) {
            // Approaching a cross-line — drive straight and slow, no PID steering
            motors_forward(APPROACH_SPEED);
        } else {
            // Normal travel between stations
            pid_compute_and_drive(false);
        }

        lcd_show_status(mission.currentStation, targetStation, "AUTO");
    }
}

// STATE_STATION_DETECT — brief confirmation pause, then mark arrival
static void handle_station_detect() {
    motors_stop();

    if (stationConfirmStatus == STN_CONFIRMED) {
        stationConfirmStatus = STN_WAITING;
        mission.currentStation = targetStation;
        sm_transition(STATE_AT_STATION);
        return;
    }

    if (stationConfirmStatus == STN_IGNORED) {
        stationConfirmStatus = STN_WAITING;
        if (targetStation > 0) stationCounter = targetStation - 1;
        ignoreCurrentIntersection = true;
        crossLineConfirmed = false;
        crossLineTimer     = 0;
        sm_transition(STATE_LINE_FOLLOW);
        return;
    }

    // No response yet — re-request confirmation if ESP32 does not reply quickly.
    if (millis() - stateTimer >= 1000UL) {
        uart_bridge_request_confirm();
        stateTimer = millis();
    }
}

// STATE_AT_STATION — announce arrival with beeps, then open wait loop
static void handle_at_station() {
    motors_stop();
    buzzer_beep_pattern(2, 150, 100); // 2 beeps: arrived at station
    lcd_show_waiting_box(mission.currentStation, BOX_TIMEOUT_MS / 1000);
    sm_transition(STATE_WAITING_BOX);
}

// STATE_WAITING_BOX — wait for operator open→close sequence or timeout
static void handle_waiting_box() {
    motors_stop();

    static bool doorWasOpen   = false; // true once door has been observed open
    static bool deliveryDone  = false; // true once full open→close cycle detected

    uint32_t elapsed = millis() - stateTimer;

    // Update countdown display once per second (non-blocking)
    static uint32_t lastLcdUpdate = 0;
    if (millis() - lastLcdUpdate >= 1000UL) {
        lastLcdUpdate = millis();
        uint16_t secsLeft = (uint16_t)((BOX_TIMEOUT_MS - elapsed) / 1000UL);
        lcd_show_waiting_box(mission.currentStation, secsLeft);
    }

    // Issue warning beep at 2.5 minutes
    if (elapsed >= BOX_WARNING_MS && !warningGiven) {
        buzzer_warning();
        warningGiven = true;
    }

    // Hard timeout — skip station
    if (elapsed >= BOX_TIMEOUT_MS) {
        doorWasOpen  = false;
        deliveryDone = false;
        sm_transition(STATE_BOX_TIMEOUT);
        return;
    }

    // Track door open→close sequence
    if (!deliveryDone) {
        if (!doorWasOpen && limit_switch_is_door_open()) {
            doorWasOpen = true; // Door first observed open
        }
        if (doorWasOpen && limit_switch_is_door_closed()) {
            deliveryDone = true; // Full cycle complete — delivery confirmed
        }
    }

    if (deliveryDone) {
        // Reset static flags for next station visit
        doorWasOpen  = false;
        deliveryDone = false;

        buzzer_beep_pattern(3, 100, 80); // 3 beeps: delivery confirmed
        mission_station_completed();

        if (mission_is_complete()) {
            sm_transition(STATE_RETURN_HOME);
        } else {
            targetStation  = mission_next_station();
            stationCounter = mission.currentStation; // count from current position
            sm_transition(STATE_LINE_FOLLOW);
        }
    }
}

// STATE_BOX_TIMEOUT — station skipped after operator inaction
static void handle_box_timeout() {
    motors_stop();
    buzzer_beep_pattern(5, 80, 60); // 5 fast beeps: skipping station
    lcd_show_message("Timeout!", "Skipping Room");
    delay(1000); // Permitted pause so operator can read the display

    mission_station_completed();

    if (mission_is_complete()) {
        // All stations visited or skipped — go home
        sm_transition(STATE_RETURN_HOME);
    } else {
        // More stations remain — continue the mission
        targetStation      = mission_next_station();
        stationCounter     = mission.currentStation; // count forward from here
        crossLineConfirmed = false;                  // clear debounce left over from timed-out station
        crossLineTimer     = 0;
        sm_transition(STATE_LINE_FOLLOW);
    }
}

// STATE_OBSTACLE — halt and alert until path is clear
static void handle_obstacle() {
    motors_stop();
    buzzer_obstacle(); // non-blocking alternating beep
    lcd_show_obstacle();

    if (!ultrasonic_obstacle_detected()) {
        // Obstacle gone — resume whichever state was interrupted
        buzzer_beep(200);
        sm_transition(savedState);
    }
}

// STATE_RETURN_HOME — follow line forward around the closed loop back to station 0
static void handle_return_home() {
    // Non-blocking crossing sub-state — used when passing intermediate stations
    enum RHCrossPhase { RH_CP_IDLE, RH_CP_WAIT_CLEAR, RH_CP_EXTRA_PUSH };
    static RHCrossPhase rhCrossPhase      = RH_CP_IDLE;
    static uint32_t     rhCrossPhaseTimer = 0;

    // Obstacle check has priority
    if (ultrasonic_obstacle_detected()) {
        motors_stop();
        sm_transition(STATE_OBSTACLE);
        return;
    }

    // While crossing an intermediate station, skip PID and detection
    if (rhCrossPhase != RH_CP_IDLE) {
        if (rhCrossPhase == RH_CP_WAIT_CLEAR) {
            // Drive forward until the robot has fully cleared the intersection line
            motors_forward(BASE_SPEED);
            IRReadings r2 = ir_read();
            if (!ir_cross_line(r2)) {
                rhCrossPhase      = RH_CP_EXTRA_PUSH;
                rhCrossPhaseTimer = millis();
            }
        } else { // RH_CP_EXTRA_PUSH
            // Push an extra 150 ms to clear the line completely before resuming PID
            motors_forward(BASE_SPEED);
            if (millis() - rhCrossPhaseTimer >= 150UL) {
                rhCrossPhase = RH_CP_IDLE;
                // Bug 2 fix: clear debounce flags so the next intersection is not
                // double-counted on the first loop iteration after crossing finishes
                crossLineConfirmed = false;
                crossLineTimer     = 0;
            }
        }
        return;
    }

    IRReadings r = ir_read();

    if (ir_cross_line(r)) {
        // Start timing the cross-line confirmation window
        if (!crossLineConfirmed) {
            crossLineConfirmed = true;
            crossLineTimer     = millis();
        } else if (millis() - crossLineTimer >= CROSS_LINE_CONFIRM_MS) {
            crossLineConfirmed = false;
            crossLineTimer     = 0;

            returnHomeCounter++;

            if (returnHomeCounter <= stationsToSkip) {
                // FIX 2: non-blocking intermediate crossing — enter RH_CP_WAIT_CLEAR
                lcd_show_message("Returning Home", "Passing stn...");
                rhCrossPhase      = RH_CP_WAIT_CLEAR;
                rhCrossPhaseTimer = millis();
            } else {
                // Passed all intermediate stations — this cross-line is station 0
                motors_stop();
                mission.currentStation = HOME_STATION;
                mission_clear();
                buzzer_beep_pattern(4, 200, 100); // 4 beeps: mission complete
                lcd_show_message("Home!", "Mission Done");
                sm_transition(STATE_IDLE);
                return;
            }
        }
        // While timing debounce window, hold position
        motors_stop();
    } else {
        // Between intersections — follow line forward
        crossLineConfirmed = false;

        // Approach detection: slow down when centre sensors see the cross-line first
        IRReadings ra = ir_read();
        if (ir_middle_black(ra) && !ir_all_black(ra)) {
            // Approaching a cross-line — drive straight and slow, no PID steering
            motors_forward(APPROACH_SPEED);
        } else {
            pid_compute_and_drive(false); // always forward on closed loop
        }

        lcd_show_home();
    }
}

// STATE_MANUAL — motor control is handled entirely by rf_drive_manual() in rf_receiver.cpp
static void handle_manual() {
    lcd_show_message("MANUAL MODE", "Joystick Active");
}

// STATE_ESTOP — all motion halted; external call to sm_transition() resumes
static void handle_estop() {
    motors_stop();
    lcd_show_estop();
}

// ─── sm_init ─────────────────────────────────────────────────────────────────

// Reset all state machine variables and enter IDLE
void sm_init() {
    currentState              = STATE_IDLE;
    stateTimer                = millis();
    warningTimer              = 0;
    warningGiven              = false;
    // Bug 1 fix: assign HOME_STATION first so stationCounter reads the correct value
    mission.currentStation    = HOME_STATION; // robot starts at station 0
    stationCounter            = HOME_STATION; // equals 0; set after currentStation is valid
    targetStation             = mission_next_station();
    crossLineConfirmed        = false;
    crossLineTimer            = 0;
    savedState                = STATE_LINE_FOLLOW;
    stationsToSkip            = 0;
    returnHomeCounter         = 0;

    sm_update_lcd(STATE_IDLE);
}

// Startup position check — non-blocking 2-second IR scan at station 0
void sm_confirm_home_position() {
    unsigned long startMs = millis();
    bool confirmed = false;

    // Poll IR sensors for up to 2 seconds without blocking indefinitely
    while (millis() - startMs < 2000UL) {
        IRReadings r = ir_read();
        if (ir_cross_line(r)) {
            confirmed = true;
            break;
        }
    }

    if (confirmed) {
        lcd_show_message("Station 0", "Confirmed :)  ");
        buzzer_beep(300);
        delay(1000);
    } else {
        lcd_show_message("WARNING!", "Place at Stn 0");
        buzzer_beep_pattern(3, 100, 100);
        delay(2000);
    }
    // Continue to STATE_IDLE regardless — startup check only
}

// ─── RF sync setters ─────────────────────────────────────────────────────────

// Allow rf_receiver.cpp to set targetStation after loading a new mission
void sm_set_target(uint8_t station) {
    targetStation = station;
}

// Allow rf_receiver.cpp to re-sync stationCounter to currentStation
void sm_set_station_counter(uint8_t val) {
    stationCounter = val;
}

// ─── sm_run ──────────────────────────────────────────────────────────────────

// Main dispatcher — must be called every loop() iteration
void sm_run() {
    switch (currentState) {
        case STATE_IDLE:            handle_idle();            break;
        case STATE_LINE_FOLLOW:     handle_line_follow();     break;
        case STATE_STATION_DETECT:  handle_station_detect();  break;
        case STATE_AT_STATION:      handle_at_station();      break;
        case STATE_WAITING_BOX:     handle_waiting_box();     break;
        case STATE_BOX_TIMEOUT:     handle_box_timeout();     break;
        case STATE_OBSTACLE:        handle_obstacle();        break;
        case STATE_RETURN_HOME:     handle_return_home();     break;
        case STATE_MANUAL:          handle_manual();          break;
        case STATE_ESTOP:           handle_estop();           break;
        default:                    motors_stop();            break;
    }
}
