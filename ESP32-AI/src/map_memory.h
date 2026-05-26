#pragma once
#include <Arduino.h>

#define MAX_SEGMENTS 10

struct MapSegment {
  uint8_t  fromStation;
  uint8_t  toStation;
  uint32_t avgTravelMs;
  uint8_t  sampleCount;
};

void     map_memory_init();
void     map_memory_record(uint8_t from, uint8_t to, uint32_t travelMs);
uint32_t map_memory_expected_ms(uint8_t from, uint8_t to);
void     map_memory_print_all();
void     map_memory_get_json(String& output);
