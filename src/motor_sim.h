#pragma once
/**
 * @file motor_sim.h
 * @brief Declarative model of the ODrowe S1 + aeration paddle. Host-testable (no Arduino).
 *
 * Everything the virtual motor does is derived from `MotorSimSpec` plus a single tick
 * engine. This keeps "what the hardware does" in one reviewable place and lets tests run
 * the exact same code path as the firmware.
 *
 * Physics (deliberately simple, protocol/contract fidelity):
 *   - first-order spin-up toward the commanded RPM (time constant from the spec)
 *   - quadratic hydrodynamic load  ibus = idle + maxLoad * (rpm / rpmMax)^2
 *   - DC bus sag                   vbus = nominal - ibus * busResistance
 *   - FET temperature              fet  = ambient + ibus * tempPerAmp (first-order toward it)
 *   - torque estimate              tau  = power / omega
 *   - ODrive state machine         IDLE(1) <-> CLOSED_LOOP_CONTROL(8), faults disarm
 */

#include <stdint.h>

/** Declarative description of the motor + driver. */
struct MotorSimSpec {
  float rpmMax = 600.0f;          // Velocity envelope (matches MAX_MOTOR_RPM)
  float spinUpTauS = 0.35f;       // First-order velocity time constant (s)
  float nominalVbus = 24.0f;      // Nominal DC bus (V)
  float busResistance = 0.035f;   // Effective source resistance (V/A)
  float idleIbus = 0.15f;         // Quiescent electronics current (A)
  float maxLoadIbus = 9.5f;       // Current at rpmMax under water load (A)
  float ambientTempC = 28.0f;     // FET ambient (degC)
  float tempPerAmp = 1.5f;        // FET rise per amp (degC/A)
  float tempTauS = 4.0f;          // FET thermal time constant (s)
  uint32_t armDelayMs = 150;      // Delay after a closed-loop request before it takes effect
};

/** Fault schedule entry: apply `mask` to axis_error once `atMs` is reached. */
struct MotorFaultEvent {
  uint32_t atMs;
  uint32_t mask;   // active_errors bitmask
};

class MotorSim {
public:
  explicit MotorSim(const MotorSimSpec& spec = MotorSimSpec());

  void reset();
  void setSpec(const MotorSimSpec& spec) { _spec = spec; }

  void requestVelocityRpm(float rpm);   // ignored unless armed
  void requestPositionTurns(float turns);
  void requestTorque(float torqueNm);
  void requestState(uint8_t state);
  void clearErrors();
  void injectFault(uint32_t mask);

  /** Advance the model. Deterministic: no internal clock reads. */
  void tick(uint32_t nowMs);

  bool isArmed() const { return _armed; }
  uint8_t state() const { return _state; }
  float actualRpm() const { return _rpm; }
  float targetRpm() const { return _targetRpm; }
  float vbus() const { return _vbus; }
  float ibus() const { return _ibus; }
  float power() const { return _power; }
  float torque() const { return _torque; }
  float fetTemp() const { return _fetTemp; }
  float iq() const { return _iq; }
  uint32_t axisError() const { return _axisError; }

  // Fault scheduling (data-driven).
  void addFaultEvent(const MotorFaultEvent& ev);
  void clearFaultSchedule() { _faultCount = 0; }

private:
  MotorSimSpec _spec;
  float _rpm = 0.0f;
  float _targetRpm = 0.0f;
  float _turns = 0.0f;
  float _vbus = 24.0f;
  float _ibus = 0.15f;
  float _power = 0.0f;
  float _torque = 0.0f;
  float _fetTemp = 28.0f;
  float _iq = 0.0f;
  uint8_t _state = 1;   // IDLE
  bool _armed = false;
  uint32_t _axisError = 0;
  uint32_t _lastMs = 0;
  uint32_t _armAtMs = 0;
  uint32_t _faultFired = 0;

  static constexpr uint8_t kMaxFaults = 8;
  MotorFaultEvent _faults[kMaxFaults] = {};
  uint8_t _faultCount = 0;
};
