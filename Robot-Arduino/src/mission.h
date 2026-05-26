#ifndef MISSION_H
#define MISSION_H

#include <Arduino.h>
#include "config.h"

// Holds the complete delivery queue and current robot position state
typedef struct {
    uint8_t queue[TOTAL_STATIONS]; // Ordered list of station numbers to visit
    uint8_t queueSize;             // Number of stations currently in the queue
    uint8_t queueIndex;            // Index of the next station to visit
    uint8_t currentStation;        // Station the robot is physically at right now
    bool    missionActive;         // True while a mission is in progress
    bool    returning_home;        // True once all deliveries are done and robot heads home
} MissionData;

// Global mission state — accessible from state_machine.cpp and robot.ino
extern MissionData mission;

// Zero out all mission data and place robot at home station
void mission_init();

// Load a station array into the queue and sort it optimally, then activate the mission
void mission_set_queue(uint8_t* stations, uint8_t count);

// Sort the queue so the robot visits stations in the most efficient order
void mission_optimize_queue();

// Return the next station to visit; returns HOME_STATION when queue is exhausted
uint8_t mission_next_station();

// Mark the current target station as delivered and advance the queue index
void mission_station_completed();

// Returns true when all queued stations have been delivered
bool mission_is_complete();

// Clear all queue data and deactivate the mission
void mission_clear();

#endif
