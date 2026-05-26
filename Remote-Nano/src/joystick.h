/*
 * ------------------------------------------------------------
 *  File:    joystick.h
 *  Purpose: Public interface for the joystick reading module.
 *
 *  This header declares the JoystickData structure and the two
 *  functions that make up the joystick module's public API.
 *  Any file that needs joystick values must include this header.
 *
 *  The module reads both joystick axes, applies a deadzone around
 *  the center position, and produces two representations of the
 *  stick position:
 *    - Signed integers (-255 to +255) for human-readable display.
 *    - Unsigned bytes (0 to 255) for packing into RF packets.
 *
 *  Dependencies:
 *    - Arduino.h : Provides int16_t, uint8_t, and other Arduino types
 *    - config.h  : JOY_X_PIN, JOY_Y_PIN, JOY_CENTER, JOY_DEADZONE
 *                  (included in joystick.cpp)
 * ------------------------------------------------------------
 */

#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <Arduino.h>

/*
 * JoystickData — holds one complete joystick reading in two formats.
 *
 * Fields:
 *   x     : Signed horizontal position after deadzone, range -255 to +255.
 *             Negative = left, 0 = center, positive = right.
 *             Used for display (the operator can read this on the LCD).
 *
 *   y     : Signed vertical position after deadzone, range -255 to +255.
 *             Negative = backward, 0 = center, positive = forward.
 *             Used for display.
 *
 *   byteX : Unsigned horizontal position mapped to 0-255 for RF packet.
 *             0 = full left, 127 = center, 255 = full right.
 *             The robot interprets 127 as "no movement" on X.
 *
 *   byteY : Unsigned vertical position mapped to 0-255 for RF packet.
 *             0 = full backward, 127 = center, 255 = full forward.
 *             The robot interprets 127 as "no movement" on Y.
 *
 * Note: x/y and byteX/byteY are both derived from the same raw ADC reading
 * after deadzone is applied, so they are always consistent with each other.
 */
typedef struct {
    int16_t x;
    int16_t y;
    uint8_t byteX;
    uint8_t byteY;
} JoystickData;

/*
 * joystick_init()
 * ---------------
 * Placeholder initialization function for the joystick module.
 * Analog pins do not require pinMode() — they are always in input mode.
 * This function exists for consistency with the other module init functions
 * and in case calibration or other setup is added in the future.
 */
void joystick_init();

/*
 * joystick_read()
 * ---------------
 * Reads both joystick axes, applies the deadzone, and returns a fully
 * populated JoystickData struct with both the signed display values
 * and the unsigned RF packet bytes.
 *
 * Returns:
 *   JoystickData — a struct containing x, y (signed, for display)
 *                  and byteX, byteY (unsigned, for RF packets).
 *
 * This function has no side effects — it only reads hardware and
 * returns a value. It is safe to call multiple times per loop.
 */
JoystickData joystick_read();

#endif
