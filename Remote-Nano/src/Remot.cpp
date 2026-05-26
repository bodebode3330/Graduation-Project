/*
 * ============================================================
 *  PIN SUMMARY — DELIVERY REMOTE (Arduino Nano)
 * ============================================================
 *
 *  ROOM BUTTONS (INPUT_PULLUP — LOW when pressed)
 *  -----------------------------------------------
 *  D2  →  Room 1 button
 *  D3  →  Room 2 button
 *  D4  →  Room 3 button
 *  D5  →  Room 4 button
 *  D6  →  Room 5 button
 *
 *  CONTROL BUTTONS (INPUT_PULLUP — LOW when pressed)
 *  --------------------------------------------------
 *  D7  →  SEND button
 *  D8  →  AUTO / MANUAL toggle button
 *  D9  →  EMERGENCY STOP button
 *
 *  JOYSTICK (Analog input)
 *  ------------------------
 *  A0  →  Joystick X axis (left=0, center≈512, right=1023)
 *  A1  →  Joystick Y axis (backward=0, center≈512, forward=1023)
 *
 *  RF TRANSMITTER MODULE (433MHz — RadioHead RH_ASK)
 *  --------------------------------------------------
 *  D12 →  RF data TX pin → connects to DATA pin on transmitter module
 *
 *  LCD I2C 16x2 (address 0x27)
 *  ----------------------------
 *  A4  →  SDA (hardware I2C)
 *  A5  →  SCL (hardware I2C)
 *
 *  POWER
 *  ------
 *  5V  →  VCC for buttons, LCD, RF module
 *  GND →  Common ground for all components
 *
 * ============================================================
 */

/*
 * ------------------------------------------------------------
 *  File:    remote.ino
 *  Purpose: Main sketch for the Arduino Nano RF delivery remote.
 *
 *  This file is the entry point of the firmware. It ties all
 *  modules together: buttons, joystick, RF transmitter, and LCD
 *  display. It manages the top-level state machine (AUTO,
 *  MANUAL, ESTOP) and coordinates what gets sent over RF and
 *  what gets shown on the screen every loop cycle.
 *
 *  Dependencies:
 *    - Wire.h              : Arduino I2C library (needed by LCD)
 *    - LiquidCrystal_I2C.h : I2C LCD driver
 *    - RH_ASK.h            : RadioHead ASK RF driver
 *    - SPI.h               : Required internally by RadioHead
 *    - config.h            : Pin definitions and timing constants
 *    - buttons.h           : Button debounce module
 *    - joystick.h          : Joystick reading module
 *    - rf_transmitter.h    : RF packet building and sending module
 *    - remote_display.h    : LCD display update module
 * ------------------------------------------------------------
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RH_ASK.h>
#include <SPI.h>
#include "config.h"
#include "buttons.h"
#include "joystick.h"
#include "rf_transmitter.h"
#include "remote_display.h"

/*
 * RF driver instance — RH_ASK handles encoding, timing, and transmission.
 * Arguments: speed=2000bps, rxPin=255 (unused — TX only), txPin=RF_TX_PIN (D12),
 *            pttPin=255 (unused — no push-to-talk control needed).
 * This object is declared here and accessed via `extern` in rf_transmitter.cpp.
 */
RH_ASK rf_driver(2000, 255, RF_TX_PIN, 255);

/*
 * LCD object — 16 columns, 2 rows, I2C address 0x27.
 * The address 0x27 is the factory default for most PCF8574-based I2C backpacks.
 * This object is declared here and accessed via `extern` in remote_display.cpp.
 */
LiquidCrystal_I2C lcd(0x27, 16, 2);

/*
 * RemoteMode — the three possible operating states of the remote.
 * Only one mode is active at a time. ESTOP has highest priority
 * and can be triggered from either AUTO or MANUAL at any time.
 */
typedef enum {
    REMOTE_AUTO,    /* Selecting rooms and dispatching delivery missions */
    REMOTE_MANUAL,  /* Joystick directly drives the robot in real time */
    REMOTE_ESTOP    /* Emergency stop — robot must halt immediately */
} RemoteMode;

/*
 * currentMode — tracks which operating mode the remote is in right now.
 * Starts in AUTO so the operator can select rooms on power-up.
 */
RemoteMode currentMode = REMOTE_AUTO;

/*
 * selectedRooms — a 5-bit bitmask of which rooms are queued for delivery.
 * Bit 0 = Room 1, bit 1 = Room 2, ..., bit 4 = Room 5.
 * Example: 0b00101 means rooms 1 and 3 are selected.
 * Starts at 0x00 meaning no rooms selected.
 */
uint8_t selectedRooms = 0x00;

/*
 * missionSent — becomes true once the operator presses SEND with at least
 * one room selected. Used to decide whether to keep re-transmitting the
 * AUTO packet and whether to show "Sent! Moving..." on the LCD.
 */
bool missionSent = false;

/*
 * lastSendTime — stores the millis() timestamp of the last RF transmission.
 * Used by handle_rf_send() to enforce the RF_SEND_INTERVAL_MS spacing between
 * packets without using delay(), which would freeze the whole program.
 */
uint32_t lastSendTime = 0;

/*
 * displayFlashEnd — stores the millis() timestamp when the room-toggle flash
 * message should expire. When a room button is pressed, the LCD briefly shows
 * "Room X: SELECTED/REMOVED" for 400ms. During that time, handle_display()
 * must not overwrite it. Once millis() passes this value, normal display resumes.
 */
uint32_t displayFlashEnd = 0;

/* Forward declarations so the compiler knows these functions exist below */
void handle_buttons();
void handle_rf_send();
void handle_display();

/*
 * setup()
 * -------
 * Runs once when the Arduino powers on or resets.
 * Initializes Serial for debug output, then initializes every module
 * in dependency order (buttons and joystick first, then RF, then LCD).
 * Ends by showing the initial AUTO screen so the operator sees something
 * immediately after the splash screen clears.
 */
void setup() {
    /* 9600 baud is sufficient for the short debug strings we print */
    Serial.begin(9600);

    /* Set up all 8 button pins as INPUT_PULLUP and initialize their structs */
    buttons_init();

    /* No hardware setup needed for analog pins, but called for consistency */
    joystick_init();

    /* Initialize the RH_ASK driver so it is ready to send packets */
    rf_transmitter_init();

    /* Initialize the LCD, show the splash screen, then clear it */
    remote_display_init();

    /* Show the initial AUTO screen — no rooms selected, not yet sent */
    remote_display_auto(selectedRooms, false);
}

/*
 * loop()
 * ------
 * Runs continuously after setup() completes — this is the main program loop.
 * Each iteration: read buttons, act on button presses, send RF if due,
 * then refresh the display. The order matters:
 *   1. buttons_update() must run first so justPressed flags are fresh.
 *   2. handle_buttons() uses those flags to update state.
 *   3. handle_rf_send() uses the updated state to decide what to transmit.
 *   4. handle_display() uses the updated state to decide what to show.
 */
void loop() {
    buttons_update();
    handle_buttons();
    handle_rf_send();
    handle_display();
}

/*
 * handle_buttons()
 * ----------------
 * Reads the justPressed flags from all buttons and updates program state.
 *
 * Priority order (highest to lowest):
 *   1. ESTOP button  — always checked first, overrides everything
 *   2. MODE button   — switches between AUTO / MANUAL / resuming from ESTOP
 *   3. Room buttons  — only active in AUTO mode
 *   4. SEND button   — only active in AUTO mode with at least one room selected
 *
 * No parameters. No return value.
 * Modifies: currentMode, missionSent, selectedRooms, displayFlashEnd.
 */
void handle_buttons() {

    /* ── ESTOP button — pin BTN_ESTOP (D9) — highest priority ─────────────────
     * Checked before anything else so it can never be blocked by other buttons.
     *
     * Behaviour:
     *   - First press (button transitions to LOW): immediately enters ESTOP mode
     *     and sends an ESTOP packet, then starts a hold timer.
     *   - Held for < RESET_HOLD_MS: shows a countdown on line 2 of the LCD.
     *   - Held for >= RESET_HOLD_MS: sends a RESET packet, shows the reset
     *     confirmation screen, then returns the remote to AUTO mode.
     *   - Released at any point: resets hold-tracking state so the next press
     *     starts a fresh ESTOP immediately.
     *
     * digitalRead() is used directly (rather than the debounced button_just_pressed
     * helper) so the hold duration can be measured continuously across many loop()
     * iterations without being edge-triggered. */
    static bool     estopHeld      = false;
    static uint32_t estopHoldStart = 0;

    if (digitalRead(BTN_ESTOP) == LOW) {
        /* Button is currently held down */
        if (!estopHeld) {
            /* First press — start the hold timer and send immediate ESTOP */
            estopHeld      = true;
            estopHoldStart = millis();
            currentMode    = REMOTE_ESTOP;
            missionSent    = false;
            rf_send_estop();
            remote_display_estop();
        } else {
            /* Still held — check if 3 seconds passed */
            uint32_t heldFor = millis() - estopHoldStart;
            if (heldFor >= RESET_HOLD_MS) {
                /* Send RESET packet */
                rf_send_reset();
                /* Show countdown on remote LCD */
                remote_display_show_reset();
                /* Reset remote state completely */
                currentMode    = REMOTE_AUTO;
                selectedRooms  = 0x00;
                missionSent    = false;
                estopHeld      = false;
                estopHoldStart = 0;
                remote_display_auto(selectedRooms, false);
            } else {
                /* Show hold progress on LCD line 2 */
                uint8_t secsLeft = (RESET_HOLD_MS - heldFor) / 1000 + 1;
                lcd.setCursor(0, 1);
                lcd.print("Hold ");
                lcd.print(secsLeft);
                lcd.print("s for RESET  ");
            }
        }
    } else {
        /* Button released — reset hold tracking */
        estopHeld      = false;
        estopHoldStart = 0;
    }

    /* ── MODE toggle button — index 6 ───────────────────────────────────────
     * Behaviour depends on what mode we are currently in:
     *   - If ESTOP is active: MODE resumes operation (goes back to AUTO)
     *   - If AUTO is active:  MODE switches to MANUAL joystick control
     *   - If MANUAL is active: MODE switches back to AUTO room selection */
    if (button_just_pressed(6)) {
        if (currentMode == REMOTE_ESTOP) {
            /* Resume from emergency stop — return to AUTO as the safe default */
            currentMode = REMOTE_AUTO;
            missionSent = false;
            remote_display_auto(selectedRooms, false);
        } else if (currentMode == REMOTE_AUTO) {
            /* Switch from room-selection mode to live joystick control */
            currentMode = REMOTE_MANUAL;
            missionSent = false;
            remote_display_manual(0, 0);  /* Show MANUAL screen with zero joystick values */
        } else if (currentMode == REMOTE_MANUAL) {
            /* Switch back to room-selection mode */
            currentMode = REMOTE_AUTO;
            missionSent = false;
            remote_display_auto(selectedRooms, false);
        }
        return; /* Only one mode action per button press */
    }

    /* ── Room buttons and SEND — only processed in AUTO mode ────────────────
     * In MANUAL mode the joystick controls the robot directly so room
     * selection makes no sense. In ESTOP mode nothing except MODE is allowed. */
    if (currentMode == REMOTE_AUTO) {

        /* Check each room button (indices 0-4 correspond to rooms 1-5) */
        for (uint8_t i = 0; i < TOTAL_ROOMS; i++) {
            if (button_just_pressed(i)) {
                /* XOR flips just bit i — if it was 0 it becomes 1, and vice versa.
                 * This toggles that room in/out of the delivery queue. */
                selectedRooms ^= (1 << i);

                /* Read back whether that room is now selected or deselected */
                bool nowSelected = (selectedRooms >> i) & 0x01;

                /* Changing the room list invalidates any previously sent mission */
                missionSent = false;

                /* Show a brief 400ms flash message confirming the toggle.
                 * displayFlashEnd is set to 400ms in the future so handle_display()
                 * knows not to overwrite the flash message until time has elapsed. */
                displayFlashEnd = millis() + 400;
                remote_display_room_toggle(i + 1, nowSelected);

                /* Stop after the first pressed room button — only process one per loop */
                break;
            }
        }

        /* ── SEND button — index 5 ──────────────────────────────────────────
         * Only sends if at least one room is selected (selectedRooms != 0x00).
         * Sending with zero rooms would dispatch the robot with no destination,
         * which is meaningless, so we silently ignore it. */
        if (button_just_pressed(5)) {
            if (selectedRooms != 0x00) {
                rf_send_auto(selectedRooms);         /* Send the mission packet immediately */
                missionSent  = true;                  /* Mark mission as dispatched */
                lastSendTime = millis();              /* Reset the send timer so handle_rf_send()
                                                         doesn't double-send within 100ms */
                remote_display_auto(selectedRooms, true); /* Switch LCD to "Sent! Moving..." */
            }
            /* If selectedRooms is 0x00, the SEND press is intentionally ignored */
        }
    }
}

/*
 * handle_rf_send()
 * ----------------
 * Sends the appropriate RF packet at regular intervals (every RF_SEND_INTERVAL_MS).
 * This function is called every loop() but internally rate-limits itself using
 * millis() so transmissions happen exactly every 100ms without using delay().
 *
 * Continuous sending is intentional — it keeps the robot updated even if a
 * packet is lost due to RF interference. The robot relies on receiving
 * periodic packets to know the remote is still alive and what to do.
 *
 * No parameters. No return value.
 */
void handle_rf_send() {
    /* If 100ms has not yet elapsed since the last send, do nothing and return.
     * This prevents flooding the RF channel and gives the robot time to process
     * each packet before the next one arrives. */
    if (millis() - lastSendTime < RF_SEND_INTERVAL_MS) return;

    /* Record the time of this transmission so the next one is spaced 100ms later */
    lastSendTime = millis();

    /* ── ESTOP: send halt command continuously ──────────────────────────────
     * Even in ESTOP mode we keep sending packets so the robot cannot miss the
     * stop command due to a single dropped packet over the RF link. */
    if (currentMode == REMOTE_ESTOP) {
        rf_send_estop();
        Serial.print("[TX] Mode:ESTP");
        Serial.print(" Mask:0x"); Serial.println(selectedRooms, HEX);
        return;
    }

    /* ── MANUAL: send live joystick position ────────────────────────────────
     * Read joystick fresh every send interval so the robot always gets the
     * most current stick position. The display reads the joystick separately
     * in handle_display(), which is fine — two reads per loop are negligible. */
    if (currentMode == REMOTE_MANUAL) {
        JoystickData joy = joystick_read();
        rf_send_manual(joy.byteX, joy.byteY);
        /* Print signed x/y values so the Serial Monitor shows direction clearly */
        Serial.print("[TX] Mode:MAN");
        Serial.print(" Mask:0x"); Serial.print(selectedRooms, HEX);
        Serial.print(" X:"); Serial.print(joy.x);
        Serial.print(" Y:"); Serial.println(joy.y);
        return;
    }

    /* ── AUTO: keep re-sending the mission packet after SEND was pressed ────
     * Once a mission is sent, we keep transmitting it so the robot continuously
     * knows it is in AUTO mode with this room queue. We only send if missionSent
     * is true — before SEND is pressed there is nothing to re-transmit. */
    if (currentMode == REMOTE_AUTO && missionSent) {
        rf_send_auto(selectedRooms);
        Serial.print("[TX] Mode:AUTO");
        Serial.print(" Mask:0x"); Serial.println(selectedRooms, HEX);
    }
}

/*
 * handle_display()
 * ----------------
 * Updates the LCD to match the current operating mode and state.
 * Called every loop() but respects the room-toggle flash window:
 * if a room button was recently pressed, the flash message is left
 * on screen until its 400ms window expires.
 *
 * No parameters. No return value.
 */
void handle_display() {
    /* If the room-toggle flash message is still within its 400ms window,
     * do not overwrite it — leave the "Room X: SELECTED/REMOVED" message visible */
    if (millis() < displayFlashEnd) return;

    /* Show the E-STOP warning screen */
    if (currentMode == REMOTE_ESTOP) {
        remote_display_estop();
        return;
    }

    /* Show live joystick X/Y values — read fresh so the display stays current */
    if (currentMode == REMOTE_MANUAL) {
        JoystickData joy = joystick_read();
        remote_display_manual(joy.x, joy.y);
        return;
    }

    /* Show the AUTO room selection screen with current selection and sent status */
    if (currentMode == REMOTE_AUTO) {
        remote_display_auto(selectedRooms, missionSent);
    }
}
