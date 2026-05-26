#include "map_memory.h"
#include <SPIFFS.h>

#define MAP_FILE "/mapdata.txt"
#define MAX_SAMPLE_COUNT 10

static MapSegment segments[MAX_SEGMENTS];
static uint8_t    segmentCount = 0;

static void load_from_spiffs() {
  if (!SPIFFS.exists(MAP_FILE)) {
    Serial.println("[MAP] No existing map file — starting fresh");
    return;
  }

  File f = SPIFFS.open(MAP_FILE, "r");
  if (!f) {
    Serial.println("[MAP] Failed to open map file for reading");
    return;
  }

  segmentCount = 0;
  while (f.available() && segmentCount < MAX_SEGMENTS) {
    MapSegment seg;
    if (f.read((uint8_t*)&seg, sizeof(MapSegment)) == sizeof(MapSegment)) {
      segments[segmentCount++] = seg;
    }
  }
  f.close();

  Serial.print("[MAP] Loaded ");
  Serial.print(segmentCount);
  Serial.println(" segment(s) from SPIFFS");
}

static void save_to_spiffs() {
  File f = SPIFFS.open(MAP_FILE, "w");
  if (!f) {
    Serial.println("[MAP] Failed to open map file for writing");
    return;
  }

  for (uint8_t i = 0; i < segmentCount; i++) {
    f.write((uint8_t*)&segments[i], sizeof(MapSegment));
  }
  f.close();

  Serial.println("[MAP] Saved to SPIFFS");
}

void map_memory_init() {
  if (!SPIFFS.begin(true)) {
    Serial.println("[MAP] SPIFFS mount failed — map memory disabled");
    return;
  }
  Serial.println("[MAP] SPIFFS mounted");
  load_from_spiffs();
}

void map_memory_record(uint8_t from, uint8_t to, uint32_t travelMs) {
  for (uint8_t i = 0; i < segmentCount; i++) {
    if (segments[i].fromStation == from && segments[i].toStation == to) {
      uint8_t n = segments[i].sampleCount;
      if (n >= MAX_SAMPLE_COUNT) n = MAX_SAMPLE_COUNT;
      segments[i].avgTravelMs = ((uint32_t)segments[i].avgTravelMs * n + travelMs) / (n + 1);
      if (segments[i].sampleCount < MAX_SAMPLE_COUNT) segments[i].sampleCount++;
      save_to_spiffs();

      Serial.print("[MAP] Updated segment ");
      Serial.print(from); Serial.print("->"); Serial.print(to);
      Serial.print(" avg="); Serial.print(segments[i].avgTravelMs);
      Serial.println("ms");
      return;
    }
  }

  if (segmentCount < MAX_SEGMENTS) {
    segments[segmentCount].fromStation  = from;
    segments[segmentCount].toStation    = to;
    segments[segmentCount].avgTravelMs  = travelMs;
    segments[segmentCount].sampleCount  = 1;
    segmentCount++;
    save_to_spiffs();

    Serial.print("[MAP] New segment ");
    Serial.print(from); Serial.print("->"); Serial.println(to);
  } else {
    Serial.println("[MAP] Segment table full — cannot add new entry");
  }
}

uint32_t map_memory_expected_ms(uint8_t from, uint8_t to) {
  for (uint8_t i = 0; i < segmentCount; i++) {
    if (segments[i].fromStation == from && segments[i].toStation == to) {
      return segments[i].avgTravelMs;
    }
  }
  return 0;
}

void map_memory_get_json(String& output) {
  output = "[";
  for (uint8_t i = 0; i < segmentCount; i++) {
    if (i > 0) output += ",";
    output += "{";
    output += "\"from\":" + String(segments[i].fromStation) + ",";
    output += "\"to\":" + String(segments[i].toStation) + ",";
    output += "\"avg\":" + String(segments[i].avgTravelMs) + ",";
    output += "\"samples\":" + String(segments[i].sampleCount);
    output += "}";
  }
  output += "]";
}

void map_memory_print_all() {
  Serial.println("[MAP] === Stored Segments ===");
  if (segmentCount == 0) {
    Serial.println("[MAP] (none)");
    return;
  }
  for (uint8_t i = 0; i < segmentCount; i++) {
    Serial.print("[MAP] ");
    Serial.print(segments[i].fromStation);
    Serial.print(" -> ");
    Serial.print(segments[i].toStation);
    Serial.print("  avg=");
    Serial.print(segments[i].avgTravelMs);
    Serial.print("ms  samples=");
    Serial.println(segments[i].sampleCount);
  }
  Serial.println("[MAP] =======================");
}
