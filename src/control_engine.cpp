#include "control_engine.h"
#include "config.h"

ControlEngine::ControlEngine() {
  _config.target_do_mg_l = DEFAULT_TARGET_DO_MG_L;
  _config.kp = DEFAULT_KP;
  _config.ki = DEFAULT_KI;
  _config.kd = DEFAULT_KD;
  _config.min_rpm = MIN_MOTOR_RPM;
  _config.max_rpm = MAX_MOTOR_RPM;
  _config.failsafe_rpm = FAILSAFE_MOTOR_RPM;
  _config.manual_throttle_pct = 0.0f;
#if defined(ODRIVE_VIRTUAL) && ODRIVE_VIRTUAL
  _config.mode = MODE_MANUAL;
#else
  _config.mode = MODE_PID;
#endif
}

void ControlEngine::configure(const ControlConfig& config) {
  portENTER_CRITICAL(&_mux);
  _config = config;
  if (_config.max_rpm < _config.min_rpm) _config.max_rpm = _config.min_rpm + 500.0f;
  _config.manual_throttle_pct = constrain(_config.manual_throttle_pct, 0.0f, 100.0f);
  portEXIT_CRITICAL(&_mux);
}

ControlConfig ControlEngine::getConfig() {
  ControlConfig copy;
  portENTER_CRITICAL(&_mux);
  copy = _config;
  portEXIT_CRITICAL(&_mux);
  return copy;
}

void ControlEngine::setManualThrottle(float throttlePct) {
  portENTER_CRITICAL(&_mux);
  _config.mode = MODE_MANUAL;
  _config.manual_throttle_pct = constrain(throttlePct, 0.0f, 100.0f);
  _emergencyStopActive = false;
  portEXIT_CRITICAL(&_mux);
}

void ControlEngine::triggerEmergencyStop() {
  portENTER_CRITICAL(&_mux);
  _config.mode = MODE_MANUAL;
  _config.manual_throttle_pct = 0.0f;
  _emergencyStopActive = true;
  portEXIT_CRITICAL(&_mux);
}

bool ControlEngine::isEmergencyStopActive() {
  bool active;
  portENTER_CRITICAL(&_mux);
  active = _emergencyStopActive;
  portEXIT_CRITICAL(&_mux);
  return active;
}

void ControlEngine::clearEmergencyStop() {
  portENTER_CRITICAL(&_mux);
  _emergencyStopActive = false;
  portEXIT_CRITICAL(&_mux);
}

void ControlEngine::setExperimentLogging(bool active) {
  portENTER_CRITICAL(&_mux);
  _experimentLogging = active;
  portEXIT_CRITICAL(&_mux);
}

bool ControlEngine::isExperimentLogging() {
  bool active;
  portENTER_CRITICAL(&_mux);
  active = _experimentLogging;
  portEXIT_CRITICAL(&_mux);
  return active;
}

void ControlEngine::updateProcessVariable(float measuredDoMgL, float waterTempC) {  portENTER_CRITICAL(&_mux);
  _lastMeasuredDo = measuredDoMgL;
  _lastWaterTemp = waterTempC;
  _lastPacketTimeMs = millis();
  _hasFirstSample = true;
  _failsafeActive = false;
  portEXIT_CRITICAL(&_mux);
}

float ControlEngine::getLastMeasuredDo() {
  float val;
  portENTER_CRITICAL(&_mux);
  val = _lastMeasuredDo;
  portEXIT_CRITICAL(&_mux);
  return val;
}

float ControlEngine::getLastWaterTemp() {
  float val;
  portENTER_CRITICAL(&_mux);
  val = _lastWaterTemp;
  portEXIT_CRITICAL(&_mux);
  return val;
}

bool ControlEngine::isFailsafeActive() {
  bool active;
  portENTER_CRITICAL(&_mux);
  active = _failsafeActive;
  portEXIT_CRITICAL(&_mux);
  return active;
}

unsigned long ControlEngine::getLastPacketAgeMs(unsigned long nowMs) {
  unsigned long age = 0xFFFFFFFFUL;
  portENTER_CRITICAL(&_mux);
  if (_hasFirstSample) {
    age = (nowMs >= _lastPacketTimeMs) ? (nowMs - _lastPacketTimeMs) : 0;
  }
  portEXIT_CRITICAL(&_mux);
  return age;
}

float ControlEngine::computeOutputRpm(unsigned long nowMs) {
  // 1. Thread-safe snapshot of state
  ControlConfig cfg;
  float measuredDo;
  bool hasSample;
  unsigned long lastPacketMs;
  bool eStop;

  portENTER_CRITICAL(&_mux);
  cfg = _config;
  measuredDo = _lastMeasuredDo;
  hasSample = _hasFirstSample;
  lastPacketMs = _lastPacketTimeMs;
  eStop = _emergencyStopActive;
  portEXIT_CRITICAL(&_mux);

  // 2. Emergency stop has top priority
  if (eStop) {
    return 0.0f;
  }

  // 3. Manual override
  if (cfg.mode == MODE_MANUAL) {
    portENTER_CRITICAL(&_mux);
    _failsafeActive = false;
    portEXIT_CRITICAL(&_mux);
    float throttle = cfg.manual_throttle_pct / 100.0f;
    return constrain(throttle * cfg.max_rpm, cfg.min_rpm, cfg.max_rpm);
  }

  // 4. Safe startup: keep motor at 0 RPM until first valid sensor packet arrives
  if (!hasSample) {
    portENTER_CRITICAL(&_mux);
    _failsafeActive = true;
    portEXIT_CRITICAL(&_mux);
    return 0.0f;
  }

  // 5. Watchdog if sensor link drops out during operation
  if (nowMs - lastPacketMs > FAILSAFE_TIMEOUT_MS) {
    portENTER_CRITICAL(&_mux);
    _failsafeActive = true;
    portEXIT_CRITICAL(&_mux);
    return cfg.failsafe_rpm;
  }

  portENTER_CRITICAL(&_mux);
  _failsafeActive = false;
  portEXIT_CRITICAL(&_mux);

  // 5. Timing calculation
  if (_lastComputeMs == 0) {
    _lastComputeMs = nowMs;
    return cfg.min_rpm;
  }
  float dt = (nowMs - _lastComputeMs) / 1000.0f;
  _lastComputeMs = nowMs;
  if (dt <= 0.0001f || dt > 5.0f) {
    dt = CONTROL_LOOP_INTERVAL_MS / 1000.0f;
  }

  // 6. Error calculation (positive error means DO is too low -> need more aeration)
  float error = cfg.target_do_mg_l - measuredDo;
  float dError = (error - _prevError) / dt;
  _prevError = error;

  float outputRpm = 0.0f;
  if (cfg.mode == MODE_PID) {
    outputRpm = runPid(error, dt, cfg);
  } else if (cfg.mode == MODE_FUZZY) {
    outputRpm = runFuzzy(error, dError, cfg);
  }

  return constrain(outputRpm, cfg.min_rpm, cfg.max_rpm);
}

float ControlEngine::runPid(float error, float dt, const ControlConfig& cfg) {
  float pTerm = cfg.kp * error;

  // Integral with anti-windup clamping
  _integral += error * dt;
  float maxIntegral = cfg.max_rpm / (cfg.ki > 0.001f ? cfg.ki : 1.0f);
  _integral = constrain(_integral, -maxIntegral, maxIntegral);
  float iTerm = cfg.ki * _integral;

  // Filtered derivative
  float dTerm = cfg.kd * (error - _prevError) / (dt > 0.001f ? dt : 0.05f);

  return pTerm + iTerm + dTerm;
}

float ControlEngine::runFuzzy(float error, float dError, const ControlConfig& cfg) {
  float u_neg = (error <= 0.0f) ? 1.0f : ((error < 0.5f) ? (0.5f - error) / 0.5f : 0.0f);
  float u_low = (error > 0.0f && error <= 0.8f) ? (error / 0.8f) :
                ((error > 0.8f && error < 1.8f) ? (1.8f - error) / 1.0f : 0.0f);
  float u_med = (error > 0.8f && error <= 1.8f) ? (error - 0.8f) / 1.0f :
                ((error > 1.8f && error < 3.0f) ? (3.0f - error) / 1.2f : 0.0f);
  float u_high = (error >= 3.0f) ? 1.0f : ((error > 1.8f) ? (error - 1.8f) / 1.2f : 0.0f);

  float u_improving = (dError <= -0.1f) ? 1.0f : ((dError < 0.0f) ? (-dError) / 0.1f : 0.0f);
  float u_steady    = (fabsf(dError) <= 0.1f) ? 1.0f - (fabsf(dError) / 0.1f) : 0.0f;
  float u_worsening = (dError >= 0.1f) ? 1.0f : ((dError > 0.0f) ? dError / 0.1f : 0.0f);

  const float RPM_OFF  = cfg.min_rpm;
  const float RPM_LOW  = cfg.min_rpm + (cfg.max_rpm - cfg.min_rpm) * 0.25f;
  const float RPM_MED  = cfg.min_rpm + (cfg.max_rpm - cfg.min_rpm) * 0.55f;
  const float RPM_HIGH = cfg.min_rpm + (cfg.max_rpm - cfg.min_rpm) * 0.80f;
  const float RPM_MAX  = cfg.max_rpm;

  float num = 0.0f;
  float den = 0.0f;

  auto addRule = [&](float weight, float singletonRpm) {
    if (weight > 0.0001f) {
      num += weight * singletonRpm;
      den += weight;
    }
  };

  addRule(u_neg, RPM_OFF);
  addRule(fminf(u_low, u_improving), RPM_OFF);
  addRule(fminf(u_low, u_steady), RPM_LOW);
  addRule(fminf(u_low, u_worsening), RPM_MED);
  addRule(fminf(u_med, u_improving), RPM_LOW);
  addRule(fminf(u_med, u_steady), RPM_MED);
  addRule(fminf(u_med, u_worsening), RPM_HIGH);
  addRule(u_high, RPM_MAX);

  if (den < 0.0001f) return cfg.min_rpm;
  return num / den;
}
