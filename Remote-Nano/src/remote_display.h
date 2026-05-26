/*
 * ------------------------------------------------------------
 *  File:    remote_display.h
 *  Purpose: Public interface for the LCD display module.
 *
 *  This header declares the functions that update the 16x2 I2C LCD
 *  screen to reflect the current operating mode and state of the remote.
 *  Any file that needs to update the display must include this header.
 *
 *  There is one function per display "screen":
 *    - Splash/init screen    : remote_display_init()
 *    - AUTO mode screen      : remote_display_auto()
 *    - MANUAL mode screen    : remote_display_manual()
 *    - ESTOP screen          : remote_display_estop()
 *    - Room toggle flash     : remote_display_room_toggle()
 *
 *  The LiquidCrystal_I2C object (lcd) is declared in remote.ino and
 *  accessed via extern in remote_display.cpp.
 *
 *  Each display function writes all 16 characters of each row it uses.
 *  Padding with spaces ensures that leftover characters from a previous
 *  longer message are always overwritten cleanly — no flicker or artifacts.
 *
 *  Dependencies:
 *    - Arduino.h          : int16_t, uint8_t, bool types
 *    - LiquidCrystal_I2C.h: LCD driver (included in remote_display.cpp)
 *    - config.h           : TOTAL_ROOMS (included in remote_display.cpp)
 * ------------------------------------------------------------
 */

#ifndef REMOTE_DISPLAY_H
#define REMOTE_DISPLAY_H

#include <Arduino.h>

/*
 * remote_display_init()
 * ---------------------
 * Initializes the LCD hardware and shows a startup splash screen for 1500ms.
 * Must be called once in setup(). This is the ONLY function in the entire
 * firmware that is allowed to use delay() — all other display updates are
 * non-blocking. After the splash, the display is cleared and ready for use.
 */
void remote_display_init();

/*
 * remote_display_auto()
 * ---------------------
 * Updates the LCD for AUTO mode, showing which rooms are selected and
 * whether a mission has been dispatched.
 *
 * Line 1: "AUTO  Rooms:XXXXX"
 *   Where XXXXX is 5 characters: the room number if selected, '-' if not.
 *   Example with rooms 1, 3, 5 selected: "AUTO  Rooms:1-3-5"
 *   Example with no rooms:               "AUTO  Rooms:-----"
 *
 * Line 2: "Press SEND      "  — if the mission has not been sent yet
 *          "Sent! Moving... "  — if the mission was dispatched
 *
 * Parameters:
 *   selectedMask : Bitmask of selected rooms (bits 0-4 = rooms 1-5).
 *   sent         : true if SEND was pressed and the mission is active.
 */
void remote_display_auto(uint8_t selectedMask, bool sent);

/*
 * remote_display_manual()
 * -----------------------
 * Updates the LCD for MANUAL mode, showing the live joystick position.
 *
 * Line 1: "MANUAL MODE     "
 * Line 2: "X: NNN Y: NNN"  — signed values, 4 characters each, right-aligned.
 *   Example: "X:  42 Y:-128"
 *
 * Parameters:
 *   x : Signed horizontal joystick position, -255 to +255.
 *   y : Signed vertical joystick position, -255 to +255.
 */
void remote_display_manual(int16_t x, int16_t y);

/*
 * remote_display_estop()
 * ----------------------
 * Updates the LCD to show the emergency stop warning screen.
 * Called immediately when ESTOP is pressed and continuously refreshed
 * while ESTOP mode is active.
 *
 * Line 1: "*** E-STOP ***  "
 * Line 2: "Press MODE:Resume"
 *
 * No parameters.
 */
void remote_display_estop();

/*
 * remote_display_room_toggle()
 * ----------------------------
 * Shows a brief confirmation message when a room button is pressed,
 * telling the operator whether that room was added to or removed from
 * the delivery queue.
 *
 * Line 1: "Room N: SELECTED" or "Room N: REMOVED "
 *   (left-aligned, padded to 16 characters)
 *
 * This function only writes line 1. Line 2 is intentionally left unchanged
 * during the flash so only the relevant information is highlighted.
 * The flash lasts for 400ms, controlled by displayFlashEnd in remote.ino.
 *
 * Parameters:
 *   room     : Room number to display, 1 to 5.
 *   selected : true if the room was just added, false if it was removed.
 */
void remote_display_room_toggle(uint8_t room, bool selected);

/*
 * remote_display_show_reset()
 * ---------------------------
 * Shows a confirmation message when a robot reset has been triggered by
 * holding the ESTOP button for RESET_HOLD_MS milliseconds.
 *
 * Line 1: "Resetting robot"
 * Line 2: "Please wait...  "
 *
 * Includes a 1000ms delay to let the operator read the message before the
 * remote transitions back to AUTO mode. This is one of only two permitted
 * uses of delay() in the firmware — the other is remote_display_init().
 *
 * No parameters.
 */
void remote_display_show_reset();

#endif
