/*
 * ------------------------------------------------------------
 *  File:    rf_transmitter.h
 *  Purpose: Public interface for the RF 433MHz transmitter module.
 *
 *  This header declares the functions used to initialize the RF driver
 *  and send packets in each of the three operating modes (AUTO, MANUAL,
 *  ESTOP). It also declares the checksum helper function.
 *
 *  All packets follow the same 5-byte format defined by the system protocol:
 *    Byte 0: MODE       (0x01=AUTO, 0x02=MANUAL, 0x03=ESTOP)
 *    Byte 1: QUEUE_MASK (room bitmask in AUTO, 0x00 otherwise)
 *    Byte 2: JOY_X      (joystick X byte in MANUAL, 0x7F otherwise)
 *    Byte 3: JOY_Y      (joystick Y byte in MANUAL, 0x7F otherwise)
 *    Byte 4: CHECKSUM   (Byte0 XOR Byte1 XOR Byte2 XOR Byte3)
 *
 *  The RH_ASK driver instance (rf_driver) is declared in remote.ino and
 *  accessed here via extern.
 *
 *  Dependencies:
 *    - Arduino.h : uint8_t and other Arduino types
 *    - RH_ASK.h  : RadioHead ASK driver (included in rf_transmitter.cpp)
 *    - SPI.h     : Required internally by RadioHead (included in rf_transmitter.cpp)
 *    - config.h  : RF_TX_PIN (included in rf_transmitter.cpp)
 * ------------------------------------------------------------
 */

#ifndef RF_TRANSMITTER_H
#define RF_TRANSMITTER_H

#include <Arduino.h>

/*
 * rf_transmitter_init()
 * ---------------------
 * Starts the RH_ASK driver and confirms readiness over Serial.
 * Must be called once in setup() before any rf_send_*() function.
 */
void rf_transmitter_init();

/*
 * rf_send_auto()
 * --------------
 * Builds and transmits an AUTO mode packet containing the room queue bitmask.
 * Used when the remote is in AUTO mode and a mission has been dispatched.
 *
 * Parameter:
 *   queueMask : Bitmask of selected rooms (bits 0-4 = rooms 1-5).
 *               Example: 0b00101 = rooms 1 and 3 selected.
 *               Range: 0x01 to 0x1F (must have at least one bit set).
 */
void rf_send_auto(uint8_t queueMask);

/*
 * rf_send_manual()
 * ----------------
 * Builds and transmits a MANUAL mode packet containing live joystick position.
 * Called every RF_SEND_INTERVAL_MS while the remote is in MANUAL mode.
 *
 * Parameters:
 *   joyX : Joystick X axis position mapped to 0-255.
 *           0 = full left, 127 (0x7F) = center, 255 = full right.
 *   joyY : Joystick Y axis position mapped to 0-255.
 *           0 = full backward, 127 (0x7F) = center, 255 = full forward.
 */
void rf_send_manual(uint8_t joyX, uint8_t joyY);

/*
 * rf_send_estop()
 * ---------------
 * Builds and transmits an ESTOP packet commanding the robot to halt immediately.
 * Called immediately when ESTOP is pressed, then continuously every
 * RF_SEND_INTERVAL_MS for as long as ESTOP mode is active.
 * No parameters — ESTOP carries no additional data.
 */
void rf_send_estop();

/*
 * rf_send_reset()
 * ---------------
 * Builds and transmits a RESET packet commanding the robot to perform a full
 * state reset. Triggered when the ESTOP button is held for RESET_HOLD_MS
 * (3 seconds) continuously. Sent once at the moment the hold threshold is met.
 *
 * Packet layout for RESET:
 *   Byte 0: 0x04   — MODE = RESET
 *   Byte 1: 0x00   — no room queue
 *   Byte 2: 0x7F   — joystick X neutral (not used in RESET)
 *   Byte 3: 0x7F   — joystick Y neutral (not used in RESET)
 *   Byte 4: checksum — XOR of bytes 0-3
 *
 * No parameters — RESET carries no variable data.
 */
void rf_send_reset();

/*
 * rf_calc_checksum()
 * ------------------
 * Computes the packet integrity checksum as XOR of the first four bytes.
 * XOR is simple and fast on an 8-bit microcontroller, and it catches any
 * single-byte corruption the receiver may verify against.
 *
 * Parameter:
 *   buf : Pointer to a 5-byte buffer. Only bytes 0-3 are used for the XOR.
 *         Byte 4 is where the result will be stored by the caller.
 *
 * Returns:
 *   uint8_t — the checksum value (buf[0] XOR buf[1] XOR buf[2] XOR buf[3]).
 */
uint8_t rf_calc_checksum(uint8_t* buf);

#endif
