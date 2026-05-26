/*
 * ------------------------------------------------------------
 *  File:    joystick.cpp
 *  Purpose: Implementation of the joystick reading module.
 *
 *  Reads the two analog axes of the joystick, applies a center deadzone
 *  to eliminate noise when the stick is at rest, then maps the cleaned
 *  reading into two formats:
 *    1. Signed -255..+255  for human-readable display on the LCD.
 *    2. Unsigned 0..255    for packing into the 5-byte RF packet.
 *
 *  The deadzone is essential because analog joystick modules rarely
 *  settle at exactly 512 ADC counts when released. Without it, the
 *  robot would receive tiny non-zero values and drift slowly even when
 *  the operator's hand is off the joystick.
 *
 *  Dependencies:
 *    - joystick.h : JoystickData struct and function declarations
 *    - config.h   : JOY_X_PIN, JOY_Y_PIN, JOY_CENTER (512), JOY_DEADZONE (40)
 * ------------------------------------------------------------
 */

#include "joystick.h"
#include "config.h"

/*
 * joystick_init()
 * ---------------
 * No hardware setup is required for analog pins — they are always inputs.
 * This function is intentionally empty but is kept so the main sketch
 * can call joystick_init() in a consistent pattern alongside buttons_init()
 * and rf_transmitter_init().
 */
void joystick_init() {
    /* Analog pins are always inputs — nothing to configure */
}

/*
 * joystick_read()
 * ---------------
 * Reads both joystick axes and returns a fully populated JoystickData struct.
 *
 * Step-by-step process:
 *   1. Read raw ADC values from both analog pins (range 0-1023).
 *   2. Apply deadzone: if the reading is within JOY_DEADZONE counts of
 *      JOY_CENTER, snap it to exactly JOY_CENTER. This eliminates small
 *      fluctuations around the stick's resting position.
 *   3. Map the cleaned raw value to signed -255..+255 for the display fields.
 *   4. Map the cleaned raw value to unsigned 0..255 for the RF packet fields.
 *   5. Return the filled struct.
 *
 * Returns:
 *   JoystickData with all four fields populated.
 */
JoystickData joystick_read() {
    JoystickData data;

    /* ── Step 1: Read raw ADC values ────────────────────────────────────────
     * analogRead() returns 0-1023 (10-bit resolution).
     * We use int16_t (signed 16-bit) so the arithmetic in step 2 can produce
     * negative intermediate values without overflow. */
    int16_t raw_x = analogRead(JOY_X_PIN);
    int16_t raw_y = analogRead(JOY_Y_PIN);

    /* ── Step 2: Apply deadzone ─────────────────────────────────────────────
     * abs(raw - CENTER) gives the distance from center.
     * If that distance is less than JOY_DEADZONE (40 counts), the stick is
     * close enough to center that we treat it as exactly centered.
     * Snapping to JOY_CENTER means the mapped output will be exactly 0 (x/y)
     * and exactly 127 (byteX/byteY) — the robot sees a clean "no input" signal.
     * Without this, electrical noise around 512 would produce tiny drift values. */
    if (abs(raw_x - JOY_CENTER) < JOY_DEADZONE) raw_x = JOY_CENTER;
    if (abs(raw_y - JOY_CENTER) < JOY_DEADZONE) raw_y = JOY_CENTER;

    /* ── Step 3: Map to signed -255..+255 for display ───────────────────────
     * Arduino's map() function linearly scales one range to another.
     * map(raw, 0, 1023, -255, 255):
     *   raw=0    → -255 (full left / full backward)
     *   raw=512  →    0 (center, after deadzone snap)
     *   raw=1023 → +255 (full right / full forward)
     * These signed values are only used for the LCD display so the operator
     * can see meaningful direction information. */
    data.x = (int16_t)map(raw_x, 0, 1023, -255, 255);
    data.y = (int16_t)map(raw_y, 0, 1023, -255, 255);

    /* ── Step 4: Map to unsigned 0..255 for RF packet bytes ─────────────────
     * map(raw, 0, 1023, 0, 255):
     *   raw=0    →   0  (full left / full backward)
     *   raw=512  → 127  (center — robot interprets this as no joystick input)
     *   raw=1023 → 255  (full right / full forward)
     * 0x7F (127) is the defined "neutral" value in the RF packet protocol.
     * Using the same raw_x/raw_y that went through the deadzone ensures that
     * a centered stick always produces exactly 127, not 126 or 128. */
    data.byteX = (uint8_t)map(raw_x, 0, 1023, 0, 255);
    data.byteY = (uint8_t)map(raw_y, 0, 1023, 0, 255);

    return data;
}
