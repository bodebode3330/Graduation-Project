#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include <Arduino.h>

// Limit switch pin — NC configuration with internal pull-up
// LOW = door closed (switch pressed), HIGH = door open (switch released)
#define LIMIT_SWITCH_PIN 13

// Configure limit switch pin with internal pull-up resistor
void limit_switch_init();

// Returns true if door is open (pin HIGH — switch not pressed)
bool limit_switch_is_door_open();

// Returns true if door is closed (pin LOW — switch pressed)
bool limit_switch_is_door_closed();

#endif
