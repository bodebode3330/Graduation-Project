#ifndef RF_RECEIVER_H
#define RF_RECEIVER_H

#include <Arduino.h>
#include <RH_ASK.h>
#include <SPI.h>

// 433MHz ASK receiver input pin (analog pin used as digital)
#define RF_RX_PIN           A0

// Packet protocol constants
#define RF_PACKET_SIZE      5      // Fixed packet length in bytes
#define JOY_DEADZONE        20     // Ignore joystick movement within ±20 of centre
#define JOY_CENTER          127    // Neutral joystick value
#define MANUAL_SPEED_MAX    200    // Maximum motor speed in manual mode (0-255)
#define RF_TIMEOUT_MS       500    // Mark packet invalid if silent for this long (ms)

// Byte 0 MODE values
typedef enum {
    RF_MODE_AUTO   = 0x01, // Autonomous line-following mission
    RF_MODE_MANUAL = 0x02, // Joystick remote control
    RF_MODE_ESTOP  = 0x03  // Emergency stop — all motion halted
} RF_Mode;

// Parsed representation of a received 5-byte RF packet
typedef struct {
    RF_Mode  mode;          // Operating mode requested by remote
    uint8_t  queueMask;     // Bitmask of requested stations (bits 0-4 = rooms 1-5)
    uint8_t  joyX;          // Joystick X axis: 0=left, 127=centre, 255=right
    uint8_t  joyY;          // Joystick Y axis: 0=back, 127=centre, 255=forward
    bool     valid;         // True if last received packet passed checksum
    uint32_t lastReceived;  // millis() timestamp of the last valid packet
} RF_Packet;

// Last successfully parsed packet — readable by state machine and debug code
extern RF_Packet lastPacket;

// Initialise RH_ASK driver and zero packet state
void rf_receiver_init();

// Non-blocking receive poll — call every loop() iteration before sm_run()
void rf_receiver_update();

// Returns true if packet checksum (XOR of bytes 0-3) matches byte 4
bool rf_checksum_valid(uint8_t* buf);

// Map a validated lastPacket to state machine transitions and motor commands
void rf_apply_packet();

// Decode a QUEUE_MASK bitmask into a station array and load it as the active mission
void rf_load_mission_from_mask(uint8_t mask);

// Translate joystick axes to mixed motor commands with deadzone and speed scaling
void rf_drive_manual(uint8_t joyX, uint8_t joyY);

#endif
