#include "adaptive_pid.h"

#define ERROR_HISTORY_SIZE  20
#define UPDATE_INTERVAL_MS  2000

#define KP_MIN  10.0f
#define KP_MAX  80.0f
#define KD_MIN  80.0f
#define KD_MAX  400.0f

static float    errorHistory[ERROR_HISTORY_SIZE];
static uint8_t  historyIdx      = 0;
static uint8_t  historyCount    = 0;
static uint32_t lastUpdateTime  = 0;

static float   currentKP;
static float   currentKD;
static uint8_t currentSpeed;

static float clampf(float val, float lo, float hi) {
  if (val < lo) return lo;
  if (val > hi) return hi;
  return val;
}

void adaptive_pid_init(float initialKP, float initialKD, uint8_t initialSpeed) {
  currentKP    = initialKP;
  currentKD    = initialKD;
  currentSpeed = initialSpeed;
  historyIdx   = 0;
  historyCount = 0;
  lastUpdateTime = 0;

  for (uint8_t i = 0; i < ERROR_HISTORY_SIZE; i++) {
    errorHistory[i] = 0.0f;
  }
}

void adaptive_pid_add_error(float error) {
  errorHistory[historyIdx] = error;
  historyIdx = (historyIdx + 1) % ERROR_HISTORY_SIZE;
  if (historyCount < ERROR_HISTORY_SIZE) historyCount++;
}

void adaptive_pid_update(HardwareSerial& port) {
  uint32_t now = millis();
  if (now - lastUpdateTime < UPDATE_INTERVAL_MS) return;
  lastUpdateTime = now;

  if (historyCount < 2) return;

  float newKP = currentKP;
  float newKD = currentKD;
  bool changed = false;

  uint8_t oscillationCount = 0;
  for (uint8_t i = 1; i < historyCount; i++) {
    uint8_t prev = (historyIdx - i     + ERROR_HISTORY_SIZE) % ERROR_HISTORY_SIZE;
    uint8_t curr = (historyIdx - i - 1 + ERROR_HISTORY_SIZE) % ERROR_HISTORY_SIZE;
    if ((errorHistory[prev] >= 0.0f) != (errorHistory[curr] >= 0.0f)) {
      oscillationCount++;
    }
  }

  if (oscillationCount > 10) {
    newKP *= 0.92f;
    changed = true;
    Serial.print("[PID] Oscillating (sign changes=");
    Serial.print(oscillationCount);
    Serial.println(") — reducing KP");
  } else if (oscillationCount < 3) {
    newKP *= 1.05f;
    changed = true;
    Serial.print("[PID] Sluggish (sign changes=");
    Serial.print(oscillationCount);
    Serial.println(") — increasing KP");
  }

  float absSum = 0.0f;
  for (uint8_t i = 0; i < historyCount; i++) {
    absSum += fabsf(errorHistory[i]);
  }
  float avgMagnitude = absSum / (float)historyCount;

  if (avgMagnitude > 2.0f) {
    newKP *= 1.05f;
    changed = true;
    Serial.print("[PID] Large persistent error (avg=");
    Serial.print(avgMagnitude, 2);
    Serial.println(") — increasing KP");
  }

  if (changed) {
    newKP = clampf(newKP, KP_MIN, KP_MAX);
    newKD = clampf(newKD, KD_MIN, KD_MAX);

    if (fabsf(newKP - currentKP) > 0.01f || fabsf(newKD - currentKD) > 0.01f) {
      currentKP = newKP;
      currentKD = newKD;

      char buf[32];
      snprintf(buf, sizeof(buf), "PID:%.1f:%.1f", currentKP, currentKD);
      port.println(buf);

      Serial.print("[PID] Sent: ");
      Serial.println(buf);
    }
  }
}

// ✨ NEW: Update PID with every frame (50ms) for faster adaptation
void adaptive_pid_update_per_frame(HardwareSerial& port) {
  if (historyCount < 5) return;  // Need at least 5 samples

  float newKP = currentKP;
  float newKD = currentKD;
  bool changed = false;

  // Check recent error trend (last 5 samples)
  float recentSum = 0.0f;
  for (uint8_t i = 0; i < 5; i++) {
    uint8_t idx = (historyIdx - 1 - i + ERROR_HISTORY_SIZE) % ERROR_HISTORY_SIZE;
    recentSum += fabsf(errorHistory[idx]);
  }
  float recentAvg = recentSum / 5.0f;

  // Check older error trend (5-10 samples ago)
  float olderSum = 0.0f;
  for (uint8_t i = 5; i < 10 && i < historyCount; i++) {
    uint8_t idx = (historyIdx - 1 - i + ERROR_HISTORY_SIZE) % ERROR_HISTORY_SIZE;
    olderSum += fabsf(errorHistory[idx]);
  }
  float olderAvg = (historyCount >= 10) ? (olderSum / 5.0f) : recentAvg;

  // If error is increasing, increase KP to respond faster
  if (recentAvg > olderAvg * 1.2f && recentAvg > 1.0f) {
    newKP *= 1.02f;  // Smaller steps for per-frame updates
    changed = true;
  }
  // If error is decreasing, slightly decrease KP to avoid overshoot
  else if (recentAvg < olderAvg * 0.8f && recentAvg < 0.5f) {
    newKP *= 0.98f;
    changed = true;
  }

  // Check for oscillation in recent samples
  uint8_t recentOscillations = 0;
  for (uint8_t i = 1; i < 5 && i < historyCount; i++) {
    uint8_t idx1 = (historyIdx - 1 - i + ERROR_HISTORY_SIZE) % ERROR_HISTORY_SIZE;
    uint8_t idx2 = (historyIdx - 1 - i - 1 + ERROR_HISTORY_SIZE) % ERROR_HISTORY_SIZE;
    if ((errorHistory[idx1] >= 0.0f) != (errorHistory[idx2] >= 0.0f)) {
      recentOscillations++;
    }
  }

  if (recentOscillations >= 3) {
    newKD *= 1.03f;  // Increase damping
    changed = true;
  }

  if (changed) {
    newKP = clampf(newKP, KP_MIN, KP_MAX);
    newKD = clampf(newKD, KD_MIN, KD_MAX);

    if (fabsf(newKP - currentKP) > 0.1f || fabsf(newKD - currentKD) > 0.5f) {
      currentKP = newKP;
      currentKD = newKD;

      char buf[32];
      snprintf(buf, sizeof(buf), "PID:%.1f:%.1f", currentKP, currentKD);
      port.println(buf);

      Serial.print("[PID-FRAME] Updated: ");
      Serial.println(buf);
    }
  }
}

// ✨ NEW: Get current PID values
void adaptive_pid_get_values(float* outKP, float* outKD, uint8_t* outSpeed) {
  if (outKP)    *outKP    = currentKP;
  if (outKD)    *outKD    = currentKD;
  if (outSpeed) *outSpeed = currentSpeed;
}

// ✨ NEW: Get speed correction based on error trend
int8_t adaptive_pid_get_speed_correction() {
  if (historyCount < 3) return 0;

  float recentSum = 0.0f;
  for (uint8_t i = 0; i < 3; i++) {
    uint8_t idx = (historyIdx - 1 - i + ERROR_HISTORY_SIZE) % ERROR_HISTORY_SIZE;
    recentSum += errorHistory[idx];
  }
  float recentAvg = recentSum / 3.0f;

  // If error is positive (drifting right), reduce speed slightly
  if (recentAvg > 1.5f) return -2;
  if (recentAvg > 0.8f) return -1;
  // If error is negative (drifting left), same correction
  if (recentAvg < -1.5f) return -2;
  if (recentAvg < -0.8f) return -1;
  
  return 0;  // No correction needed
}
