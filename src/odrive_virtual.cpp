#include "odrive_virtual.h"
#include "config.h"
#include <math.h>

ODriveVirtual::ODriveVirtual(float nominalVbus, float rampRateRpmPerSec)
    : _nominalVbus(nominalVbus),
      _rampRate(rampRateRpmPerSec),
      _targetRpm(0.0f),
      _actualRpm(0.0f),
      _vbus(nominalVbus),
      _ibus(VIRTUAL_IBUS_IDLE),
      _power(0.0f),
      _state(1),        // AXIS_STATE_IDLE
      _controlMode(2),  // VELOCITY_CONTROL
      _inputMode(2),    // VEL_RAMP
      _axisError(0),
      _isArmed(false),
      _lastUpdateMs(0) {}

bool ODriveVirtual::begin() {
  _state = 1; // IDLE
  _isArmed = false;
  _axisError = 0;
  _targetRpm = 0.0f;
  _actualRpm = 0.0f;
  _vbus = _nominalVbus;
  _ibus = VIRTUAL_IBUS_IDLE;
  _power = _vbus * _ibus;
  _lastUpdateMs = millis();
  Serial.println("[ODrive Virtual] Driver initialized. Simulation active.");
  return true;
}

bool ODriveVirtual::setVelocityRpm(float rpm) {
  if (!_isArmed || _state != 8) {
    _targetRpm = 0.0f;
    return true;
  }
  _targetRpm = constrain(rpm, MIN_MOTOR_RPM, MAX_MOTOR_RPM);
  return true;
}

bool ODriveVirtual::setPosition(float /*turns*/) {
  return true;
}

bool ODriveVirtual::setTorque(float /*torqueNm*/) {
  return true;
}

bool ODriveVirtual::setControlMode(uint8_t controlMode, uint8_t inputMode) {
  _controlMode = controlMode;
  _inputMode = inputMode;
  return true;
}

bool ODriveVirtual::requestState(uint8_t state) {
  _state = state;
  _isArmed = (state == 8); // 8 = CLOSED_LOOP_CONTROL
  if (!_isArmed) {
    _targetRpm = 0.0f;
  }
  return true;
}

bool ODriveVirtual::enterClosedLoop() {
  clearErrors();
  _state = 8;
  _isArmed = true;
  return true;
}

bool ODriveVirtual::enterIdle() {
  _state = 1;
  _isArmed = false;
  _targetRpm = 0.0f;
  return true;
}

bool ODriveVirtual::clearErrors() {
  _axisError = 0;
  return true;
}

void ODriveVirtual::setSimulatedError(uint32_t errorCode) {
  _axisError = errorCode;
  if (errorCode != 0) {
    enterIdle();
  }
}

void ODriveVirtual::setNominalVbus(float vbus) {
  _nominalVbus = vbus;
}

void ODriveVirtual::setRampRate(float rpmPerSec) {
  if (rpmPerSec > 10.0f) {
    _rampRate = rpmPerSec;
  }
}

void ODriveVirtual::updatePhysics(unsigned long nowMs) {
  if (_lastUpdateMs == 0) {
    _lastUpdateMs = nowMs;
    return;
  }

  float dt = (float)(nowMs - _lastUpdateMs) / 1000.0f;
  _lastUpdateMs = nowMs;

  if (dt <= 0.0f) return;
  if (dt > 0.5f) dt = 0.5f; // ponytail: clamp large pause jumps (e.g. WiFi reconnect blocking)

  float prevRpm = _actualRpm;

  // 1. Inertial acceleration / deceleration towards target setpoint
  if (!_isArmed || _state != 8) {
    // Coast down when disarmed / idle
    float coastRamp = _rampRate * 1.5f * dt;
    if (_actualRpm > coastRamp) {
      _actualRpm -= coastRamp;
    } else {
      _actualRpm = 0.0f;
    }
  } else {
    float maxDelta = _rampRate * dt;
    if (_targetRpm > _actualRpm) {
      _actualRpm = fminf(_actualRpm + maxDelta, _targetRpm);
    } else if (_targetRpm < _actualRpm) {
      _actualRpm = fmaxf(_actualRpm - maxDelta, _targetRpm);
    }
  }

  // 2. Realistic load current & voltage sag
  // Idle baseline: ~0.15A
  // Water aeration paddle hydrodynamic load is quadratic with rotational speed:
  float normSpeed = _actualRpm / MAX_MOTOR_RPM;
  float loadCurrent = VIRTUAL_IBUS_MAX_LOAD * (normSpeed * normSpeed);

  // Acceleration dynamic current
  float dRpmDt = fabsf(_actualRpm - prevRpm) / dt;
  float accelCurrent = dRpmDt * 0.0015f;

  _ibus = VIRTUAL_IBUS_IDLE + loadCurrent + accelCurrent;

  // DC bus voltage sag under current: Vbus = Vnom - (Rinternal * Ibus)
  _vbus = _nominalVbus - (_ibus * 0.035f);
  _power = _vbus * _ibus;
}

bool ODriveVirtual::pollTelemetry(ODriveTelemetry& outTelem) {
  updatePhysics(millis());

  outTelem.vbus_voltage = _vbus;
  outTelem.ibus_current = _ibus;
  outTelem.actual_rpm = _actualRpm;
  outTelem.power_watts = _power;
  outTelem.axis_error = _axisError;
  outTelem.current_state = _state;
  outTelem.control_mode = _controlMode;
  outTelem.input_mode = _inputMode;
  outTelem.is_armed = _isArmed;

  return true;
}

bool ODriveVirtual::readFloatProperty(const char* prop, float& outVal, uint32_t /*timeoutMs*/) {
  updatePhysics(millis());
  if (!prop) return false;

  if (strstr(prop, "vbus_voltage")) {
    outVal = _vbus;
    return true;
  }
  if (strstr(prop, "ibus")) {
    outVal = _ibus;
    return true;
  }
  if (strstr(prop, "vel_estimate")) {
    outVal = _actualRpm / 60.0f; // turns/sec
    return true;
  }
  return false;
}

bool ODriveVirtual::writeFloatProperty(const char* prop, float val) {
  if (!prop) return false;
  if (strstr(prop, "input_vel")) {
    setVelocityRpm(val * 60.0f);
    return true;
  }
  return true;
}

bool ODriveVirtual::sendRawCommand(const char* cmd, char* outBuf, size_t maxLen, uint32_t /*timeoutMs*/) {
  if (!cmd) return false;

  if (strcmp(cmd, "sc") == 0) {
    clearErrors();
    return true;
  }

  if (outBuf && maxLen > 0) {
    if (strstr(cmd, "vbus_voltage")) {
      snprintf(outBuf, maxLen, "%.2f", _vbus);
      return true;
    }
    if (strstr(cmd, "active_errors")) {
      snprintf(outBuf, maxLen, "%u", _axisError);
      return true;
    }
    if (strstr(cmd, "current_state")) {
      snprintf(outBuf, maxLen, "%u", _state);
      return true;
    }
    if (strstr(cmd, "control_mode")) {
      snprintf(outBuf, maxLen, "%u", _controlMode);
      return true;
    }
    if (strstr(cmd, "input_mode")) {
      snprintf(outBuf, maxLen, "%u", _inputMode);
      return true;
    }
    outBuf[0] = '\0';
  }

  return true;
}
