#include "rf_receiver.h"
#include "config.h"
#include "mission.h"
#include "state_machine.h"
#include "motors.h"

// ─── RH_ASK driver instance ──────────────────────────────────────────────────
// speed=2000 bps, rxPin=A0, txPin=255 (unused), pttPin=255 (unused)
static RH_ASK rf_driver(2000, RF_RX_PIN, 255, 255);

// ─── Public packet state ──────────────────────────────────────────────────────
RF_Packet lastPacket;

// ─── rf_receiver_init ────────────────────────────────────────────────────────

// Initialise the RH_ASK driver and zero the packet struct
void rf_receiver_init() {
    rf_driver.init();

    lastPacket.mode         = RF_MODE_AUTO;
    lastPacket.queueMask    = 0;
    lastPacket.joyX         = JOY_CENTER;
    lastPacket.joyY         = JOY_CENTER;
    lastPacket.valid        = false;
    lastPacket.lastReceived = 0;
}

// ─── rf_checksum_valid ───────────────────────────────────────────────────────

// Verify packet integrity: XOR of bytes 0-3 must equal byte 4
bool rf_checksum_valid(uint8_t* buf) {
    return (buf[0] ^ buf[1] ^ buf[2] ^ buf[3]) == buf[4];
}

// ─── rf_receiver_update ──────────────────────────────────────────────────────

// Non-blocking receive poll — must be the first call in loop()
void rf_receiver_update() {
    uint8_t buf[RF_PACKET_SIZE];
    uint8_t bufLen = RF_PACKET_SIZE;

    if (rf_driver.recv(buf, &bufLen)) {
        // Discard packets with unexpected length
        if (bufLen != RF_PACKET_SIZE) return;

        // Discard silently on checksum failure
        if (!rf_checksum_valid(buf)) return;

        // Parse validated packet fields
        lastPacket.mode         = (RF_Mode)buf[0];
        lastPacket.queueMask    = buf[1];
        lastPacket.joyX         = buf[2];
        lastPacket.joyY         = buf[3];
        lastPacket.valid        = true;
        lastPacket.lastReceived = millis();

        rf_apply_packet();

    } else {
        // No packet received — check for RF signal timeout
        if (lastPacket.valid &&
            (millis() - lastPacket.lastReceived > RF_TIMEOUT_MS)) {
            lastPacket.valid = false;
        }
    }
}

// ─── rf_load_mission_from_mask ───────────────────────────────────────────────

// Decode a QUEUE_MASK bitmask into an ordered station array and activate mission
void rf_load_mission_from_mask(uint8_t mask) {
    uint8_t stations[TOTAL_STATIONS];
    uint8_t count = 0;

    // Bits 0-4 correspond to rooms 1-5
    for (uint8_t i = 0; i < TOTAL_STATIONS; i++) {
        if (mask & (1 << i)) {
            stations[count++] = i + 1;
        }
    }

    mission_set_queue(stations, count);

    // Sync state machine internals to the freshly loaded mission
    sm_set_target(mission_next_station());
    sm_set_station_counter(mission.currentStation);
}

// ─── rf_drive_manual ─────────────────────────────────────────────────────────

// Map joystick X/Y axes to motor speeds with deadzone, mixing, and clamping
void rf_drive_manual(uint8_t joyX, uint8_t joyY) {
    // Apply deadzone: treat axis as centred if within ±JOY_DEADZONE of JOY_CENTER
    bool yActive = (abs((int)joyY - JOY_CENTER) >= JOY_DEADZONE);
    bool xActive = (abs((int)joyX - JOY_CENTER) >= JOY_DEADZONE);

    // Special case: spin in place when only X axis is active
    if (xActive && !yActive) {
        if (joyX > JOY_CENTER + JOY_DEADZONE) {
            motors_spin_right(MANUAL_SPEED_MAX / 2);
        } else {
            motors_spin_left(MANUAL_SPEED_MAX / 2);
        }
        return;
    }

    // Y axis: compute base forward/backward speed
    int baseSpeed  = 0;
    int yDirection = 0; // +1 = forward, -1 = backward

    if (yActive) {
        if (joyY > JOY_CENTER + JOY_DEADZONE) {
            // Joystick forward = robot backward (inverted)
            baseSpeed  = (int)map(joyY, JOY_CENTER, 255, 0, MANUAL_SPEED_MAX);
            yDirection = -1;
        } else {
            // Joystick backward = robot forward (inverted)
            baseSpeed  = (int)map(joyY, JOY_CENTER, 0, 0, MANUAL_SPEED_MAX);
            yDirection = +1;
        }
    }

    // X axis: compute steering turn offset
    int turnOffset = 0;
    if (xActive) {
        int xMag   = abs((int)joyX - JOY_CENTER);
        turnOffset = (int)map(xMag, JOY_DEADZONE, 127, 0, MANUAL_SPEED_MAX / 2);
        if (joyX < JOY_CENTER - JOY_DEADZONE) turnOffset = -turnOffset; // left
    }

    // Mix Y speed and X steering into per-side motor commands
    int leftSpeed  = yDirection * baseSpeed - turnOffset;
    int rightSpeed = yDirection * baseSpeed + turnOffset;

    // Clamp to valid manual speed range (negative = reverse)
    leftSpeed  = constrain(leftSpeed,  -MANUAL_SPEED_MAX, MANUAL_SPEED_MAX);
    rightSpeed = constrain(rightSpeed, -MANUAL_SPEED_MAX, MANUAL_SPEED_MAX);

    motors_set(leftSpeed, rightSpeed);
}

// ─── rf_apply_packet ─────────────────────────────────────────────────────────

// Map the validated lastPacket to state machine transitions and motor actions
void rf_apply_packet() {

    // ── ESTOP — highest priority, overrides any active state immediately ──────
    if (lastPacket.mode == RF_MODE_ESTOP) {
        if (currentState != STATE_ESTOP) {
            motors_stop();
            sm_transition(STATE_ESTOP);
        }
        return;
    }

    // ── MANUAL MODE ───────────────────────────────────────────────────────────
    if (lastPacket.mode == RF_MODE_MANUAL) {
        if (currentState != STATE_MANUAL && currentState != STATE_ESTOP) {
            motors_stop();
            sm_transition(STATE_MANUAL);
        }
        if (currentState == STATE_MANUAL) {
            rf_drive_manual(lastPacket.joyX, lastPacket.joyY);
        }
        return;
    }

    // ── AUTO MODE ─────────────────────────────────────────────────────────────
    if (lastPacket.mode == RF_MODE_AUTO) {

        // Resume from ESTOP → return to IDLE for re-evaluation
        if (currentState == STATE_ESTOP) {
            sm_transition(STATE_IDLE);
            return;
        }

        // Switching from MANUAL back to AUTO
        if (currentState == STATE_MANUAL) {
            motors_stop();

            // Restore previous mission if one is still in progress
            if (mission.queueSize > 0 && !mission_is_complete()) {
                sm_set_station_counter(mission.currentStation);
                sm_transition(STATE_LINE_FOLLOW);
            } else {
                // Load a new mission from the mask if one is provided
                if (lastPacket.queueMask != 0) {
                    rf_load_mission_from_mask(lastPacket.queueMask);
                    sm_transition(STATE_LINE_FOLLOW);
                } else {
                    sm_transition(STATE_IDLE);
                }
            }
            return;
        }

        // IDLE with a new queue mask → start a fresh mission
        if (currentState == STATE_IDLE) {
            if (lastPacket.queueMask != 0) {
                rf_load_mission_from_mask(lastPacket.queueMask);
                sm_transition(STATE_LINE_FOLLOW);
            }
            return;
        }

        // All other AUTO states — state machine handles itself
    }
}
