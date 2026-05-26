/*
 * ------------------------------------------------------------
 *  File:    remote_display.cpp
 *  Purpose: Implementation of the LCD display module.
 *
 *  This module handles all text output to the 16x2 I2C LCD screen.
 *  It provides one function per "screen" that the remote can show.
 *  Each function formats its text carefully to always fill the full
 *  16-character width of each row — trailing spaces overwrite any
 *  leftover characters from a previous longer message, preventing
 *  display corruption without needing to call lcd.clear() (which
 *  causes a visible flicker).
 *
 *  The lcd object is declared in remote.ino and accessed here via extern.
 *  The display module never modifies program state — it is output-only.
 *
 *  Dependencies:
 *    - remote_display.h   : Function declarations
 *    - config.h           : TOTAL_ROOMS (number of rooms to display)
 *    - LiquidCrystal_I2C.h: LCD driver for the Frank de Brabander library
 * ------------------------------------------------------------
 */

#include "remote_display.h"
#include "config.h"
#include <LiquidCrystal_I2C.h>

/*
 * lcd — the LiquidCrystal_I2C instance created in remote.ino.
 * Declared extern so this file can call lcd.print(), lcd.setCursor(), etc.
 * without creating a second object that would conflict on the I2C bus.
 */
extern LiquidCrystal_I2C lcd;

/*
 * remote_display_init()
 * ---------------------
 * Powers on the LCD, turns on the backlight, and displays a startup splash
 * screen for 1500ms to let the operator know the remote is booting.
 *
 * delay(1500) is the only permitted use of delay() in the entire firmware.
 * It is safe here because setup() is still running — the main loop has not
 * started yet, so there is nothing else that needs to happen concurrently.
 * After the delay, lcd.clear() wipes the splash so other functions start
 * with a clean display.
 */
void remote_display_init() {
    /* Start the LCD hardware and enable the I2C backlight */
    lcd.init();
    lcd.backlight();

    /* Show the splash screen on both rows */
    lcd.setCursor(0, 0);
    lcd.print("Delivery Remote ");   /* Row 0 — product name, padded to 16 chars */
    lcd.setCursor(0, 1);
    lcd.print("Initializing... ");   /* Row 1 — status message, padded to 16 chars */

    /* Hold the splash for 1500ms so the operator can read it.
     * 1500ms was chosen as long enough to read but short enough not to feel slow. */
    delay(1500);

    /* Clear the display completely before normal operation begins */
    lcd.clear();
}

/*
 * remote_display_auto()
 * ---------------------
 * Renders the AUTO mode screen showing the room selection bitmask and
 * whether the mission has been dispatched.
 *
 * Line 1 format: "AUTO  Rooms:XXXXX"
 *   The 5-character room field maps each bit of selectedMask to a character:
 *     bit set   → character '1' through '5' (the room number)
 *     bit clear → '-' (room not selected)
 *   Example: mask=0b00101 (rooms 1 and 3) → "1-3--"
 *
 * Line 2: prompt changes based on whether SEND has been pressed yet.
 *
 * Parameters:
 *   selectedMask : Bitmask of which rooms are in the delivery queue.
 *   sent         : true after SEND was pressed, false before.
 */
void remote_display_auto(uint8_t selectedMask, bool sent) {
    /* line1 holds 16 printable characters + null terminator */
    char line1[17];

    /* rooms[] holds the 5-character room status string + null terminator */
    char rooms[6];

    /* ── Build the room status string ───────────────────────────────────────
     * Iterate over each room index (0-4 for rooms 1-5).
     * Check whether bit i of selectedMask is set using a bitwise AND after
     * right-shifting: (selectedMask >> i) & 0x01 gives 1 if selected, 0 if not.
     * '1' + i gives the ASCII character for the room number (e.g., i=2 → '3'). */
    for (uint8_t i = 0; i < TOTAL_ROOMS; i++) {
        if ((selectedMask >> i) & 0x01) {
            rooms[i] = '1' + i;   /* Room is selected — show its number */
        } else {
            rooms[i] = '-';        /* Room is not selected — show a dash */
        }
    }
    rooms[5] = '\0';   /* Null-terminate the string so snprintf treats it as a C string */

    /* Format line 1: "AUTO  Rooms:" followed by the 5-char rooms string.
     * snprintf limits output to 16 characters + null terminator, preventing overflow. */
    snprintf(line1, sizeof(line1), "AUTO  Rooms:%s", rooms);

    /* Write line 1 starting at column 0, row 0 */
    lcd.setCursor(0, 0);
    lcd.print(line1);

    /* ── Write line 2 based on mission status ───────────────────────────────
     * Both strings are exactly 16 characters (including trailing spaces) so
     * they always overwrite the full row cleanly. */
    lcd.setCursor(0, 1);
    if (!sent) {
        /* Prompt the operator to press SEND when ready */
        lcd.print("Press SEND      ");
    } else {
        /* Confirm that the mission was dispatched and the robot is moving */
        lcd.print("Sent! Moving... ");
    }
}

/*
 * remote_display_manual()
 * -----------------------
 * Renders the MANUAL mode screen showing the current joystick position.
 *
 * Line 1: static label "MANUAL MODE     " — padded to 16 chars.
 * Line 2: formatted "X: NNN Y: NNN" — 4-character wide fields for X and Y,
 *   right-aligned with leading spaces so the numbers don't jump around.
 *   Example: "X:  42 Y:-128"
 *
 * Parameters:
 *   x : Signed joystick X, -255 to +255. Negative = left, positive = right.
 *   y : Signed joystick Y, -255 to +255. Negative = backward, positive = forward.
 */
void remote_display_manual(int16_t x, int16_t y) {
    char line2[17];

    /* Line 1 is a static label — write it directly without formatting */
    lcd.setCursor(0, 0);
    lcd.print("MANUAL MODE     ");   /* Trailing spaces fill the full 16-char width */

    /* Format line 2 with right-aligned 4-digit signed fields.
     * "%4d" pads the number to 4 characters wide, right-aligned.
     * Casting to int is needed because snprintf's %d expects int, not int16_t. */
    snprintf(line2, sizeof(line2), "X:%4d Y:%4d", (int)x, (int)y);

    lcd.setCursor(0, 1);
    lcd.print(line2);
}

/*
 * remote_display_estop()
 * ----------------------
 * Renders the emergency stop warning screen.
 * Both lines are written every time this is called, which happens
 * continuously while ESTOP mode is active, so the screen is always correct.
 *
 * Line 1: "*** E-STOP ***  " — asterisks make this visually alarming.
 * Line 2: "Press MODE:Resume" — tells the operator exactly how to recover.
 *
 * Both strings are padded or sized to fill the 16-character row width.
 */
void remote_display_estop() {
    lcd.setCursor(0, 0);
    lcd.print("*** E-STOP ***  ");    /* Visual alert — asterisks signal danger */
    lcd.setCursor(0, 1);
    lcd.print("Press MODE:Resume");   /* Recovery instruction */
}

/*
 * remote_display_room_toggle()
 * ----------------------------
 * Shows a brief confirmation on line 1 of the LCD when a room button is pressed.
 * This gives the operator immediate feedback that their button press was registered.
 *
 * Line 1 format: "Room N: SELECTED" or "Room N: REMOVED "
 *   "%-8s" in snprintf means left-align the string in an 8-character field.
 *   "SELECTED" is exactly 8 chars; "REMOVED " is padded with a trailing space
 *   to also be 8 chars — both fill the field so no leftover characters remain.
 *
 * Only line 1 is written here. Line 2 is intentionally left unchanged so the
 * room queue summary on line 2 remains visible during the flash.
 *
 * Parameters:
 *   room     : The room number (1-5) to display in the message.
 *   selected : true if the room was just added to the queue, false if removed.
 */
/*
 * remote_display_show_reset()
 * ---------------------------
 * Renders the reset confirmation screen when a robot reset is triggered.
 * Called once from handle_buttons() in remote.ino immediately after
 * rf_send_reset() is sent and before the remote returns to AUTO mode.
 *
 * Line 1: "Resetting robot" — informs the operator a reset is in progress.
 * Line 2: "Please wait...  " — padded to 16 characters for a clean write.
 *
 * delay(1000) is permitted here (matching the splash screen precedent) because
 * the operator needs time to read the message and the 1-second pause is a
 * natural acknowledgement that a significant action just occurred.
 */
void remote_display_show_reset() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Resetting robot");
    lcd.setCursor(0, 1);
    lcd.print("Please wait...  ");
    delay(1000);
}

void remote_display_room_toggle(uint8_t room, bool selected) {
    char line1[17];

    /* Build the message — "SELECTED" (8 chars) or "REMOVED " (8 chars, space-padded)
     * so the full 16-character line is always written cleanly */
    snprintf(line1, sizeof(line1), "Room %d: %-8s", room, selected ? "SELECTED" : "REMOVED ");

    lcd.setCursor(0, 0);
    lcd.print(line1);

    /* Line 2 is intentionally NOT written here.
     * The caller (handle_buttons in remote.ino) sets displayFlashEnd to millis()+400,
     * so handle_display() will leave line 2 alone for the 400ms flash duration. */
}
