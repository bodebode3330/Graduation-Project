#include "mission.h"

// Global mission state instance
MissionData mission;

// Zero out all mission data and set robot to home position
void mission_init() {
    for (uint8_t i = 0; i < TOTAL_STATIONS; i++) {
        mission.queue[i] = 0;
    }
    mission.queueSize      = 0;
    mission.queueIndex     = 0;
    mission.currentStation = HOME_STATION;
    mission.missionActive  = false;
    mission.returning_home = false;
}

// Load stations into queue, optimize ordering, then activate the mission
void mission_set_queue(uint8_t* stations, uint8_t count) {
    // Clamp to max capacity to prevent overflow
    if (count > TOTAL_STATIONS) count = TOTAL_STATIONS;

    mission.queueSize  = count;
    mission.queueIndex = 0;

    for (uint8_t i = 0; i < count; i++) {
        mission.queue[i] = stations[i];
    }

    mission_optimize_queue();

    mission.missionActive  = true;
    mission.returning_home = false;
}

// Sort queue for optimal traversal: stations ahead first (ascending),
// then stations behind (ascending). Implements a simple insertion sort.
void mission_optimize_queue() {
    uint8_t n = mission.queueSize;
    if (n == 0) return;

    // --- Step 1: insertion sort ascending ---
    for (uint8_t i = 1; i < n; i++) {
        uint8_t key = mission.queue[i];
        int8_t  j   = (int8_t)i - 1;
        while (j >= 0 && mission.queue[j] > key) {
            mission.queue[j + 1] = mission.queue[j];
            j--;
        }
        mission.queue[j + 1] = key;
    }

    // If starting from home (station 0), ascending order is already optimal
    if (mission.currentStation == HOME_STATION) return;

    // --- Step 2: rotate so that stations AHEAD come first ---
    // Find the first index where queue[i] > currentStation
    uint8_t splitIdx = n; // default: all stations are behind
    for (uint8_t i = 0; i < n; i++) {
        if (mission.queue[i] > mission.currentStation) {
            splitIdx = i;
            break;
        }
    }

    // If all stations are ahead or all are behind, no rotation needed
    if (splitIdx == 0 || splitIdx == n) return;

    // Rotate: [splitIdx..n-1] + [0..splitIdx-1]
    uint8_t temp[TOTAL_STATIONS];
    uint8_t idx = 0;
    for (uint8_t i = splitIdx; i < n; i++) temp[idx++] = mission.queue[i];
    for (uint8_t i = 0;        i < splitIdx; i++) temp[idx++] = mission.queue[i];
    for (uint8_t i = 0; i < n; i++) mission.queue[i] = temp[i];
}

// Return the next station to visit; returns HOME_STATION when queue is exhausted
uint8_t mission_next_station() {
    if (mission.queueIndex >= mission.queueSize) return HOME_STATION;
    return mission.queue[mission.queueIndex];
}

// Advance queue index; flag returning_home and deactivate if all done
void mission_station_completed() {
    mission.queueIndex++;
    if (mission.queueIndex >= mission.queueSize) {
        mission.returning_home = true;
        mission.missionActive  = false;
    }
}

// Returns true when every queued station has been visited
bool mission_is_complete() {
    return mission.queueIndex >= mission.queueSize;
}

// Reset all queue and mission state without reinitialising currentStation
void mission_clear() {
    for (uint8_t i = 0; i < TOTAL_STATIONS; i++) {
        mission.queue[i] = 0;
    }
    mission.queueSize      = 0;
    mission.queueIndex     = 0;
    mission.missionActive  = false;
    mission.returning_home = false;
}
