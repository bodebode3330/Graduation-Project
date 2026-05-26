#include "ir_sensors.h"
#include "motors.h"

// Read all four IR sensors and return a snapshot struct
IRReadings ir_read() {
    IRReadings r;
    r.farLeft     = digitalRead(IR_FAR_LEFT)     == HIGH;
    r.centerLeft  = digitalRead(IR_CENTER_LEFT)  == HIGH;
    r.centerRight = digitalRead(IR_CENTER_RIGHT) == HIGH;
    r.farRight    = digitalRead(IR_FAR_RIGHT)    == HIGH;
    return r;
}

// Returns true when all four sensors read black — cross-line detected
bool ir_all_black(IRReadings r) {
    return r.farLeft && r.centerLeft && r.centerRight && r.farRight;
}

// Returns true when the robot is over a strong cross-line signature.
// This allows smoother detection when the line is not wide enough to trigger
// all four sensors simultaneously.
bool ir_cross_line(IRReadings r) {
    return ir_all_black(r) || (ir_middle_black(r) && (r.farLeft || r.farRight));
}

// Returns true when the two middle sensors read black — station detection.
bool ir_middle_black(IRReadings r) {
    return r.centerLeft && r.centerRight;
}

// Returns true when all four sensors read white — line completely lost
bool ir_all_white(IRReadings r) {
    return !r.farLeft && !r.centerLeft && !r.centerRight && !r.farRight;
}

// Compute weighted position error for PID; weights: -3, -1, +1, +3
int ir_compute_error(IRReadings r) {
    // Retain last known error when the line is lost to allow recovery
    static int lastError = 0;

    // If line completely lost, return last known error to continue recovering
    if (ir_all_white(r)) {
        return lastError;
    }

    // Weighted sum and active sensor count for weighted average
    int weightedSum = 0;
    int activeCount = 0;

    if (r.farLeft)     { weightedSum += -3; activeCount++; }
    if (r.centerLeft)  { weightedSum += -1; activeCount++; }
    if (r.centerRight) { weightedSum += +1; activeCount++; }
    if (r.farRight)    { weightedSum += +3; activeCount++; }

    // Guard against divide-by-zero (should not happen since all-white is caught above)
    if (activeCount == 0) return lastError;

    // Weighted average error; round toward nearest integer
    int error = weightedSum / activeCount;
    lastError = error;
    return error;
}

// ─── PID state ───────────────────────────────────────────────────────────────

static float         pid_integral  = 0.0f;
static int           pid_lastError = 0;
static unsigned long pid_lastTime  = 0;
static float         pid_kp        = PID_KP;
static float         pid_ki        = PID_KI;
static float         pid_kd        = PID_KD;
static uint8_t       pid_baseSpeed = BASE_SPEED;

// Reset PID state — call before starting line-following to avoid stale data
void pid_init() {
    pid_integral  = 0.0f;
    pid_lastError = 0;
    pid_lastTime  = millis();
}

void pid_set_kp(float kp) {
    pid_kp = kp;
}

void pid_set_kd(float kd) {
    pid_kd = kd;
}

void pid_set_base_speed(uint8_t speed) {
    pid_baseSpeed = speed;
}

// Read IR sensors, compute PID correction, and drive motors
// reverse=false → forward travel; reverse=true → backward travel (return home)
void pid_compute_and_drive(bool reverse) {
    IRReadings r = ir_read();

    // Station detected — stop and let higher-level logic take over
    if (ir_all_black(r)) {
        motors_stop();
        return;
    }

    // Compute elapsed time since last PID iteration in milliseconds
    unsigned long now     = millis();
    float         deltaMs = (float)(now - pid_lastTime);

    // Guard against zero or negative delta (clock wrap or first call)
    if (deltaMs <= 0.0f) deltaMs = 1.0f;

    // Current position error from line centre
    int error = ir_compute_error(r);

    // Proportional term
    float proportional = pid_kp * (float)error;

    // Integral term — accumulate and clamp to prevent wind-up
    pid_integral += pid_ki * (float)error * deltaMs;
    pid_integral  = constrain(pid_integral, -100.0f, 100.0f);

    // Derivative term — rate of error change per millisecond
    float derivative = pid_kd * (float)(error - pid_lastError) / deltaMs;

    // Combined PID output (steering correction)
    float output = proportional + pid_integral + derivative;

    int leftSpeed;
    int rightSpeed;

    if (!reverse) {
        // Forward: left gets +correction, right gets -correction
        leftSpeed  = (int)((float)pid_baseSpeed + output);
        rightSpeed = (int)((float)pid_baseSpeed - output);
        leftSpeed  = constrain(leftSpeed,  0, MAX_SPEED);
        rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);
        motors_set(leftSpeed, rightSpeed);
    } else {
        // Reverse: invert correction sides so steering still tracks the line
        leftSpeed  = (int)((float)pid_baseSpeed - output);
        rightSpeed = (int)((float)pid_baseSpeed + output);
        leftSpeed  = constrain(leftSpeed,  0, MAX_SPEED);
        rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);
        motors_set(-leftSpeed, -rightSpeed);
    }

    // Save state for next iteration
    pid_lastError = error;
    pid_lastTime  = now;
}
