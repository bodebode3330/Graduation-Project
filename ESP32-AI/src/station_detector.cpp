#include "station_detector.h"

#define HISTORY_SIZE     20
#define EVAL_WINDOW       6
#define CONFIRM_THRESHOLD 0.70f
#define IMPROVING_WINDOW   3

static FilteredIR irHistory[HISTORY_SIZE];
static uint8_t    historyHead   = 0;
static uint8_t    historyCount  = 0;

static uint8_t  confirmedCount    = 0;
static uint32_t lastConfirmTime   = 0;
static uint32_t crossingDuration  = 0;

// Track current confidence for progressive detection
static float currentConfidence = 0.0f;
static uint8_t pattern_consistency = 0;  // How many consecutive frames with high IR

void station_detector_init() {
  historyHead  = 0;
  historyCount = 0;
  confirmedCount   = 0;
  lastConfirmTime  = 0;
  crossingDuration = 0;
  currentConfidence = 0.0f;
  pattern_consistency = 0;

  for (uint8_t i = 0; i < HISTORY_SIZE; i++) {
    irHistory[i] = {0.0f, 0.0f, 0.0f, 0.0f};
  }
}

void station_detector_add_sample(FilteredIR sample) {
  irHistory[historyHead] = sample;
  historyHead = (historyHead + 1) % HISTORY_SIZE;
  if (historyCount < HISTORY_SIZE) historyCount++;
  
  // Update pattern consistency: count if all 4 sensors are > 0.6
  float avgValue = (sample.farLeft + sample.centerLeft + 
                    sample.centerRight + sample.farRight) / 4.0f;
  if (avgValue > 0.6f) {
    if (pattern_consistency < 255) pattern_consistency++;
  } else {
    pattern_consistency = 0;
  }
  
  // Calculate current confidence (rolling average of last 3 samples)
  float scoreSum = 0.0f;
  uint8_t checkCount = (historyCount < IMPROVING_WINDOW) ? historyCount : IMPROVING_WINDOW;
  for (uint8_t i = 0; i < checkCount; i++) {
    uint8_t idx = (historyHead - 1 - i + HISTORY_SIZE) % HISTORY_SIZE;
    FilteredIR& s = irHistory[idx];
    scoreSum += (s.farLeft + s.centerLeft + s.centerRight + s.farRight) / 4.0f;
  }
  currentConfidence = scoreSum / (float)checkCount;
}

// ✨ NEW: Get current confidence level (0.0 - 1.0)
float station_detector_get_confidence() {
  return currentConfidence;
}

// ✨ NEW: Check if pattern is stabilizing (confidence improving)
bool station_detector_is_improving() {
  return pattern_consistency >= 3;  // At least 3 consecutive high readings
}

bool station_detector_evaluate(HardwareSerial& port) {
  if (historyCount < EVAL_WINDOW) {
    port.println("STN:IGNORE");
    Serial.println("[STN] Not enough history — IGNORE");
    return false;
  }

  float scoreSum = 0.0f;
  uint8_t available = (historyCount < EVAL_WINDOW) ? historyCount : EVAL_WINDOW;

  for (uint8_t i = 0; i < available; i++) {
    uint8_t idx = (historyHead - 1 - i + HISTORY_SIZE) % HISTORY_SIZE;
    FilteredIR& s = irHistory[idx];
    scoreSum += (s.farLeft + s.centerLeft + s.centerRight + s.farRight) / 4.0f;
  }

  float score = scoreSum / (float)available;

  Serial.print("[STN] Confidence score: ");
  Serial.println(score, 3);

  if (score >= CONFIRM_THRESHOLD) {
    port.println("STN:CONFIRM");
    confirmedCount++;
    uint32_t now = millis();
    if (lastConfirmTime > 0) {
      crossingDuration = now - lastConfirmTime;
    }
    lastConfirmTime = now;
    Serial.print("[STN] CONFIRM — total confirmed: ");
    Serial.println(confirmedCount);
    pattern_consistency = 0;  // Reset after confirmation
    return true;
  } else {
    port.println("STN:IGNORE");
    Serial.println("[STN] IGNORE — score below threshold");
    return false;
  }
}
