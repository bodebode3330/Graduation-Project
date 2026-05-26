/*
 * ------------------------------------------------------------
 *  File:    rf_transmitter.cpp
 *  Purpose: Implementation of the RF 433MHz transmitter module.
 *
 *  This module builds 5-byte RF packets and sends them using the
 *  RadioHead RH_ASK library. There are three packet types — one for
 *  each operating mode (AUTO, MANUAL, ESTOP) — all sharing the same
 *  5-byte structure defined by the system protocol.
 *
 *  Packet format (5 bytes):
 *    [0] MODE byte  : 0x01=AUTO, 0x02=MANUAL, 0x03=ESTOP
 *    [1] QUEUE_MASK : room bitmask (AUTO only), 0x00 otherwise
 *    [2] JOY_X      : joystick X byte (MANUAL only), 0x7F otherwise
 *    [3] JOY_Y      : joystick Y byte (MANUAL only), 0x7F otherwise
 *    [4] CHECKSUM   : XOR of bytes 0-3 for basic error detection
 *
 *  The RH_ASK driver (rf_driver) and LCD object are declared in remote.ino.
 *  This file accesses rf_driver via extern to avoid duplicate instantiation.
 *
 *  Dependencies:
 *    - rf_transmitter.h : Function declarations
 *    - config.h         : RF_TX_PIN
 *    - RH_ASK.h         : RadioHead ASK RF driver
 *    - SPI.h            : Required internally by RadioHead
 * ------------------------------------------------------------
 */

#include "rf_transmitter.h"
#include "config.h"
#include <RH_ASK.h>
#include <SPI.h>

/*
 * rf_driver — the RH_ASK instance created in remote.ino.
 * Declared extern here so this file can call rf_driver.send() and
 * rf_driver.waitPacketSent() without creating a second driver object,
 * which would cause pin conflicts and double initialization.
 */
extern RH_ASK rf_driver;

/*
 * rf_calc_checksum()
 * ------------------
 * Computes a simple XOR checksum across the first four bytes of the packet.
 *
 * XOR is chosen because:
 *   - It is a single instruction on AVR (the ATmega328P CPU in the Nano).
 *   - XOR of all bytes is 0 if no bits have flipped — easy for the receiver
 *     to verify by XOR-ing all 5 bytes and checking the result equals 0.
 *
 * Parameter:
 *   buf : Pointer to the packet buffer. Must contain at least 4 bytes (0-3).
 *
 * Returns:
 *   uint8_t — XOR of buf[0], buf[1], buf[2], buf[3].
 */
uint8_t rf_calc_checksum(uint8_t* buf) {
    return buf[0] ^ buf[1] ^ buf[2] ^ buf[3];
}

/*
 * rf_transmitter_init()
 * ---------------------
 * Initializes the RH_ASK driver so it is ready to send packets.
 * Must be called once in setup() before any rf_send_*() function.
 *
 * rf_driver.init() configures the TX pin (D12) and sets up the
 * internal timer that RH_ASK uses for bit timing at 2000 bps.
 */
void rf_transmitter_init() {
    rf_driver.init();
    /* Confirm readiness on the Serial monitor so the developer knows
     * the RF module started without errors during testing */
    Serial.println("RF Transmitter ready");
}

/*
 * rf_send_auto()
 * --------------
 * Builds and sends an AUTO mode packet.
 * This tells the robot: "You are in AUTO mode; deliver to these rooms."
 *
 * Packet layout for AUTO:
 *   Byte 0: 0x01        — MODE = AUTO
 *   Byte 1: queueMask   — which rooms are in the delivery queue
 *   Byte 2: 0x7F        — joystick X neutral (not used in AUTO mode)
 *   Byte 3: 0x7F        — joystick Y neutral (not used in AUTO mode)
 *   Byte 4: checksum    — XOR of bytes 0-3
 *
 * 0x7F is the neutral/center value (127 out of 255) for joystick bytes.
 * The robot ignores bytes 2-3 in AUTO mode, but we send neutral values
 * so a malfunctioning robot that reads them anyway sees "no movement".
 *
 * Parameter:
 *   queueMask : Bitmask of selected rooms. Bit 0 = Room 1, bit 4 = Room 5.
 */
void rf_send_auto(uint8_t queueMask) {
    uint8_t buf[5];

    /* Build the packet */
    buf[0] = 0x01;        /* AUTO mode identifier */
    buf[1] = queueMask;   /* Which rooms to deliver to */
    buf[2] = 0x7F;        /* Joystick X neutral — not applicable in AUTO */
    buf[3] = 0x7F;        /* Joystick Y neutral — not applicable in AUTO */
    buf[4] = rf_calc_checksum(buf); /* XOR of bytes 0-3 */

    /* Send the packet and block until the entire transmission finishes.
     * waitPacketSent() ensures the full packet is on the air before we return.
     * This prevents us from overwriting the buffer or calling send() again
     * before the previous transmission is complete. */
    rf_driver.send(buf, 5);
    rf_driver.waitPacketSent();
}

/*
 * rf_send_manual()
 * ----------------
 * Builds and sends a MANUAL mode packet containing live joystick position.
 * This tells the robot: "You are in MANUAL mode; move according to the stick."
 *
 * Packet layout for MANUAL:
 *   Byte 0: 0x02   — MODE = MANUAL
 *   Byte 1: 0x00   — no room queue in manual mode
 *   Byte 2: joyX   — joystick X position (0=left, 127=center, 255=right)
 *   Byte 3: joyY   — joystick Y position (0=back,  127=center, 255=forward)
 *   Byte 4: checksum — XOR of bytes 0-3
 *
 * Parameters:
 *   joyX : Joystick X axis, 0-255. From JoystickData.byteX.
 *   joyY : Joystick Y axis, 0-255. From JoystickData.byteY.
 */
void rf_send_manual(uint8_t joyX, uint8_t joyY) {
    uint8_t buf[5];

    /* Build the packet */
    buf[0] = 0x02;   /* MANUAL mode identifier */
    buf[1] = 0x00;   /* No room queue in MANUAL mode */
    buf[2] = joyX;   /* Live joystick X position */
    buf[3] = joyY;   /* Live joystick Y position */
    buf[4] = rf_calc_checksum(buf); /* XOR of bytes 0-3 */

    /* Transmit and wait for completion before returning */
    rf_driver.send(buf, 5);
    rf_driver.waitPacketSent();
}

/*
 * rf_send_reset()
 * ---------------
 * Builds and sends a RESET packet commanding the robot to perform a full
 * state reset. Called once when the ESTOP button has been held for
 * RESET_HOLD_MS milliseconds continuously.
 *
 * Packet layout for RESET:
 *   Byte 0: 0x04   — MODE = RESET  (RF_MODE_RESET from config.h)
 *   Byte 1: 0x00   — no room queue
 *   Byte 2: 0x7F   — joystick X neutral (not used in RESET)
 *   Byte 3: 0x7F   — joystick Y neutral (not used in RESET)
 *   Byte 4: checksum — XOR of bytes 0-3
 *
 * No parameters — RESET carries no variable data.
 */
void rf_send_reset() {
    uint8_t buf[5];

    /* Build the reset packet */
    buf[0] = 0x04;   /* RESET mode identifier (RF_MODE_RESET) */
    buf[1] = 0x00;   /* No room queue */
    buf[2] = 0x7F;   /* Joystick X neutral */
    buf[3] = 0x7F;   /* Joystick Y neutral */
    buf[4] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3]; /* XOR checksum */

    /* Transmit and wait for completion before returning */
    rf_driver.send(buf, 5);
    rf_driver.waitPacketSent();
}

/*
 * rf_send_estop()
 * ---------------
 * Builds and sends an ESTOP packet commanding the robot to stop immediately.
 * This tells the robot: "EMERGENCY STOP — halt all motion right now."
 *
 * Packet layout for ESTOP:
 *   Byte 0: 0x03   — MODE = ESTOP
 *   Byte 1: 0x00   — no room queue
 *   Byte 2: 0x7F   — joystick X neutral (not used in ESTOP)
 *   Byte 3: 0x7F   — joystick Y neutral (not used in ESTOP)
 *   Byte 4: checksum — XOR of bytes 0-3
 *
 * Joystick bytes are set to 0x7F (neutral) rather than 0x00 so that a robot
 * receiving a partial or misread packet does not interpret it as full movement.
 * No parameters — ESTOP carries no variable data.
 */
void rf_send_estop() {
    uint8_t buf[5];

    /* Build the packet */
    buf[0] = 0x03;   /* ESTOP mode identifier */
    buf[1] = 0x00;   /* No room queue */
    buf[2] = 0x7F;   /* Joystick X neutral */
    buf[3] = 0x7F;   /* Joystick Y neutral */
    buf[4] = rf_calc_checksum(buf); /* XOR of bytes 0-3 */

    /* Transmit and wait for completion before returning */
    rf_driver.send(buf, 5);
    rf_driver.waitPacketSent();
}
