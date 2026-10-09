#include "motor_sim.h"

#include <math.h>

MotorSim::MotorSim(const MotorSimSpec& spec) : _spec(spec) {
  reset();
}

void MotorSim::reset() {
  _rpm = 0.0f;
  _targetRpm = 0.0f;
  _turns = 0.0f;
  _vbus = _spec.nominalVbus;
  _ibus = _spec.idleIbus;
  _power = _vbus * _ibus;
  _torque = 0.0f;
  _fetTemp = _spec.ambientTempC;
  _iq = 0.0f;
  _state = 1;  // IDLE
  _armed = false;
  _axisError = 0;
  _lastMs = 0;
  _armAtMs = 0;
  _faultFired = 0;
}

void MotorSim::requestVelocityRpm(float rpm) {
  if (!_armed || _state != 8) {
    _targetRpm = 0.0f;
    return;
  }
  if (rpm < 0.0f) rpm = 0.0f;
  if (rpm > _spec.rpmMax) rpm = _spec.rpmMax;
  _targetRpm = rpm;
}

void MotorSim::requestPositionTurns(float turns) {
  _turns = turns;
}

void MotorSim::requestTorque(float torqueNm) {
  // Torque mode is unused by the node; keep the value for completeness.
  _torque = torqueNm;
}

void MotorSim::requestState(uint8_t state) {
  _state = state;
  _armed = (state == 8);
  if (_armed) {
    // Arm after a short delay (mirrors the ODrive's closed-loop entry latency).
    _armAtMs = _lastMs + _spec.armDelayMs;
  } else {
    _targetRpm = 0.0f;
  }
}

void MotorSim::clearErrors() {
  _axisError = 0;
  _faultFired = 0;
}

void MotorSim::injectFault(uint32_t mask) {
  _axisError |= mask;
  // A real ODrive leaves closed loop on a fault.
  _state = 1;
  _armed = false;
  _targetRpm = 0.0f;
}

void MotorSim::addFaultEvent(const MotorFaultEvent& ev) {
  if (_faultCount < kMaxFaults) {
    _faults[_faultCount++] = ev;
  }
}

void MotorSim::tick(uint32_t nowMs) {
  if (_lastMs == 0) {
    _lastMs = nowMs;
    return;
  }
  float dt = (float)(nowMs - _lastMs) / 1000.0f;
  _lastMs = nowMs;
  if (dt <= 0.0f) return;
  if (dt > 0.5f) dt = 0.5f;  // clamp long pauses (e.g. WiFi reconnect)

  // --- Data-driven faults --------------------------------------------------
  for (uint8_t i = 0; i < _faultCount; ++i) {
    const bool fired = (_faultFired & (1u << i)) != 0;
    if (!fired && nowMs >= _faults[i].atMs) {
      _faultFired |= (1u << i);
      injectFault(_faults[i].mask);
    }
  }

  const bool closedLoop = _armed && _state == 8;
  const bool armSettled = closedLoop && nowMs >= _armAtMs;

  const float prevRpm = _rpm;

  // --- Velocity dynamics ---------------------------------------------------
  if (armSettled) {
    const float alpha = dt / (_spec.spinUpTauS + dt);
    _rpm += (_targetRpm - _rpm) * alpha;
  } else {
    // Coast down toward zero when idle / disarmed.
    const float alpha = dt / (_spec.spinUpTauS + dt);
    _rpm += (0.0f - _rpm) * alpha;
    if (_rpm < 0.5f) _rpm = 0.0f;
  }
  if (_rpm > _spec.rpmMax) _rpm = _spec.rpmMax;

  // Position integrates velocity.
  _turns += (_rpm / 60.0f) * dt;

  // --- Load, bus, thermal --------------------------------------------------
  const float norm = (_spec.rpmMax > 0.0f) ? (_rpm / _spec.rpmMax) : 0.0f;
  const float loadCurrent = _spec.maxLoadIbus * norm * norm;
  _ibus = _spec.idleIbus + loadCurrent;

  _vbus = _spec.nominalVbus - _ibus * _spec.busResistance;
  _power = _vbus * _ibus;

  // Mechanical torque from shaft power and angular speed. Below the load floor the
  // shaft is essentially unloaded, so report zero rather than amplifying the small
  // idle power over a near-zero omega (which produced implausible iq spikes at stop).
  const float omega = (_rpm * 2.0f * 3.14159265f) / 60.0f;
  _torque = (omega > 1.0f) ? (_power / omega) : 0.0f;

  // q-axis current proxy: torque / (kT) with kT ~ 0.09 Nm/A for this class of motor.
  // Keep the same threshold as torque so the two stay physically consistent.
  _iq = (omega > 1.0f) ? (_torque / 0.09f) : 0.0f;

  const float fetTarget = _spec.ambientTempC + _ibus * _spec.tempPerAmp;
  const float ta = dt / (_spec.tempTauS + dt);
  _fetTemp += (fetTarget - _fetTemp) * ta;

  (void)prevRpm;
}
