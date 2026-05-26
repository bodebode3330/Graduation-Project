/*
 * ------------------------------------------------------------
 *  File:    buttons.h
 *  Purpose: Public interface for the button debounce module.
 *
 *  This header declares the Button data structure and the three
 *  functions that the rest of the firmware uses to interact with
 *  buttons. Any file that needs to read button state must include
 *  this header.
 *
 *  This module manages all 8 buttons:
 *    Indices 0-4  →  Room 1 through Room 5 selection buttons
 *    Index  5     →  SEND button
 *    Index  6     →  MODE toggle button
 *    Index  7     →  EMERGENCY STOP button
 *
 *  Dependencies:
 *    - Arduino.h : Provides uint8_t, bool, and other Arduino types
 *    - config.h  : Pin number defines (included in buttons.cpp)
 * ------------------------------------------------------------
 */

#ifndef BUTTONS_H
#define BUTTONS_H

#include <Arduino.h>

/*
 * Button — holds all state needed to debounce one physical button.
 *
 * Fields:
 *   pin             : The Arduino digital pin number this button is wired to.
 *   lastState       : The raw pin reading from the previous loop iteration.
 *                     Used to detect when the reading changes (which resets
 *                     the debounce timer).
 *   currentState    : The confirmed, debounced state of the button.
 *                     Only updated after the reading has been stable for
 *                     at least DEBOUNCE_MS milliseconds.
 *   lastDebounceTime: The millis() timestamp of the last time the raw reading
 *                     changed. The debounce timer measures from this point.
 *   justPressed     : Set to true for exactly ONE loop iteration when the
 *                     button transitions from released (HIGH) to pressed (LOW).
 *                     Automatically reset to false at the start of the next
 *                     buttons_update() call, so callers can rely on it being
 *                     a single-shot event flag.
 */
typedef struct {
    uint8_t  pin;
    bool     lastState;
    bool     currentState;
    uint32_t lastDebounceTime;
    bool     justPressed;
} Button;

/*
 * buttons_init()
 * --------------
 * Initializes all 8 buttons: sets every pin to INPUT_PULLUP and
 * fills in each Button struct with its starting state.
 * Must be called once in setup() before any other button function.
 */
void buttons_init();

/*
 * buttons_update()
 * ----------------
 * Reads every button pin and updates the debounce state machine.
 * Must be called once at the very start of every loop() iteration
 * so that justPressed flags are fresh when handle_buttons() checks them.
 */
void buttons_update();

/*
 * button_just_pressed()
 * ---------------------
 * Returns true if the button at the given index was pressed (falling edge
 * detected and debounced) during this loop iteration, false otherwise.
 *
 * Parameter:
 *   buttonIndex : 0 = Room 1, 1 = Room 2, 2 = Room 3, 3 = Room 4,
 *                 4 = Room 5, 5 = SEND, 6 = MODE, 7 = ESTOP.
 *                 Values >= 8 safely return false.
 *
 * Returns:
 *   bool — true only for the single loop iteration in which the press
 *           was confirmed. False every other time.
 */
bool button_just_pressed(uint8_t buttonIndex);

#endif
