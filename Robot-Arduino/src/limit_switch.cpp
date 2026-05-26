#include "limit_switch.h"

// Configure limit switch pin with the internal pull-up resistor enabled
void limit_switch_init() {
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
}

// Returns true when the door is open (switch released — pin reads HIGH)
bool limit_switch_is_door_open() {
    return digitalRead(LIMIT_SWITCH_PIN) == HIGH;
}

// Returns true when the door is closed (switch pressed — pin reads LOW)
bool limit_switch_is_door_closed() {
    return digitalRead(LIMIT_SWITCH_PIN) == LOW;
}
