/*
 * ------------------------------------------------------------
 *  File:    buttons.cpp
 *  Purpose: Implementation of the button debounce module.
 *
 *  This module maintains an array of Button structs — one per physical
 *  button — and runs a debounce state machine on every loop iteration.
 *  The debounce logic prevents false triggers caused by the rapid
 *  electrical bouncing that all mechanical switches produce when pressed.
 *
 *  The algorithm used here is "timer-based debounce":
 *    - Every time the raw pin reading changes, restart a timer.
 *    - Only accept the new reading as valid once it has held stable
 *      for DEBOUNCE_MS milliseconds without changing again.
 *    - If the newly confirmed state is LOW (button pressed), set justPressed.
 *
 *  Dependencies:
 *    - buttons.h : Button struct definition and function declarations
 *    - config.h  : DEBOUNCE_MS, BTN_ROOM_1..5, BTN_SEND, BTN_MODE, BTN_ESTOP
 * ------------------------------------------------------------
 */

#include "buttons.h"
#include "config.h"

/*
 * buttons[] — the internal array holding the live state of all 8 buttons.
 * Declared static so it is private to this file — no other file can access
 * it directly. All external access goes through the three public functions.
 */
static Button buttons[8];

/*
 * buttonPins[] — maps each button index to its physical Arduino pin number.
 * Index 0-4 = Room 1-5, Index 5 = SEND, Index 6 = MODE, Index 7 = ESTOP.
 * Declared const because these pin assignments never change at runtime.
 * Declared static so it is private to this file.
 */
static const uint8_t buttonPins[8] = {
    BTN_ROOM_1, BTN_ROOM_2, BTN_ROOM_3, BTN_ROOM_4, BTN_ROOM_5,
    BTN_SEND, BTN_MODE, BTN_ESTOP
};

/*
 * buttons_init()
 * --------------
 * Sets up every button's pin as INPUT_PULLUP and initializes its struct
 * to a known starting state — button released, no debounce pending.
 *
 * Must be called once in setup() before buttons_update() is ever called.
 * Calling this after setup() would reset all button states unexpectedly.
 */
void buttons_init() {
    for (uint8_t i = 0; i < 8; i++) {
        /* Store the pin number so the rest of the module knows which pin to read */
        buttons[i].pin = buttonPins[i];

        /* Buttons use INPUT_PULLUP, so the resting state is HIGH (not pressed).
         * We initialize both lastState and currentState to HIGH to match reality
         * and prevent a false "just pressed" event on the very first loop. */
        buttons[i].lastState    = HIGH;
        buttons[i].currentState = HIGH;

        /* No debounce is in progress yet — timer starts from 0 */
        buttons[i].lastDebounceTime = 0;

        /* No press has occurred yet */
        buttons[i].justPressed = false;

        /* Enable the internal pull-up resistor on this pin.
         * This means the pin reads HIGH when the button is open (not pressed)
         * and LOW when the button shorts the pin to GND (pressed).
         * No external resistor is needed in this wiring configuration. */
        pinMode(buttonPins[i], INPUT_PULLUP);
    }
}

/*
 * buttons_update()
 * ----------------
 * Runs the debounce state machine for every button. Must be called
 * once at the start of every loop() so justPressed flags reflect
 * button activity from the current iteration only.
 *
 * How the debounce works (per button):
 *   1. Reset justPressed from the previous iteration to false.
 *   2. Read the raw pin state right now.
 *   3. If the raw state is different from last iteration, restart the timer.
 *   4. If the raw state has been stable for >= DEBOUNCE_MS:
 *      a. If it differs from the confirmed state, update the confirmed state.
 *      b. If the newly confirmed state is LOW (button pressed), set justPressed.
 *   5. Save the raw state as lastState for comparison next iteration.
 */
void buttons_update() {
    for (uint8_t i = 0; i < 8; i++) {

        /* Clear justPressed so it is only true for one loop iteration per press.
         * This must happen first — before we check whether to set it again. */
        buttons[i].justPressed = false;

        /* Read the raw electrical state of the pin right now */
        bool reading = digitalRead(buttons[i].pin);

        /* If the raw reading changed since last iteration, the button may be
         * bouncing. Reset the debounce timer — we need it to be stable for
         * another DEBOUNCE_MS milliseconds before trusting the new reading. */
        if (reading != buttons[i].lastState) {
            buttons[i].lastDebounceTime = millis();
        }

        /* Only proceed if the reading has been stable for the full debounce period.
         * If the timer hasn't elapsed yet, we simply wait and do nothing. */
        if ((millis() - buttons[i].lastDebounceTime) >= DEBOUNCE_MS) {

            /* The reading is now stable. Check if it is different from what we
             * last confirmed — if so, the button genuinely changed state. */
            if (reading != buttons[i].currentState) {
                buttons[i].currentState = reading;

                /* LOW means the button is now pressed (pulled to GND).
                 * This is the falling edge (HIGH → LOW) we care about.
                 * We do NOT set justPressed on a rising edge (release). */
                if (buttons[i].currentState == LOW) {
                    buttons[i].justPressed = true;
                }
            }
        }

        /* Save the raw reading so next iteration can detect if it changes */
        buttons[i].lastState = reading;
    }
}

/*
 * button_just_pressed()
 * ---------------------
 * Returns whether a specific button was freshly pressed this loop iteration.
 *
 * Parameter:
 *   buttonIndex : Index into the buttons[] array (0-7).
 *                 0 = Room 1, 1 = Room 2, 2 = Room 3, 3 = Room 4,
 *                 4 = Room 5, 5 = SEND, 6 = MODE, 7 = ESTOP.
 *
 * Returns:
 *   true  — button was pressed (falling edge confirmed) this loop iteration.
 *   false — button was not pressed, or index is out of range.
 *
 * The bounds check (buttonIndex >= 8) prevents out-of-bounds array access
 * if the caller accidentally passes an invalid index.
 */
bool button_just_pressed(uint8_t buttonIndex) {
    /* Guard against out-of-bounds access — silently return false for bad indices */
    if (buttonIndex >= 8) return false;

    return buttons[buttonIndex].justPressed;
}
