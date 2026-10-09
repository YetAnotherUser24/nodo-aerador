/**
 * @file motor_bus_virtual.cpp
 * @brief Virtual `IMotorBus`: binds the declarative MotorSim to the node interface.
 *
 * Build: selected by `-DAERADOR_VIRTUAL=1` (env `esp32-s3-virtual`). Never compiled into
 * real firmware. Talk to the real backend; no ESP-NOW.
 */

#include "motor_bus.h"
#include "motor_sim.h"
#include "config.h"

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

namespace {

class VirtualMotorBus : public IMotorBus {
public:
  VirtualMotorBus() {
    MotorSimSpec spec;
    spec.rpmMax = MAX_MOTOR_RPM;
    spec.nominalVbus = VIRTUAL_VBUS_NOMINAL;
    spec.idleIbus = VIRTUAL_IBUS_IDLE;
    spec.maxLoadIbus = VIRTUAL_IBUS_MAX_LOAD;
    _sim.setSpec(spec);
    // Deterministic, demonstrable fault schedule (disabled unless VIRTUAL_FAULTS=1).
#if defined(AERADOR_VIRTUAL_FAULTS) && AERADOR_VIRTUAL_FAULTS
    _sim.addFaultEvent({15000, 0x00000001u});  // INITIALIZING-ish fault at 15 s
    _sim.addFaultEvent({25000, 0x00000002u});  // another at 25 s
#endif
  }

  void tick(uint32_t nowMs) override { _sim.tick(nowMs); }

  bool begin() override {
    _sim.reset();
    _specApplied = true;
    Serial.println("[MotorBus/Virtual] ODrive S1 simulation active.");
    return true;
  }

  bool setVelocityRpm(float rpm) override {
    _sim.requestVelocityRpm(rpm);
    return true;
  }
  bool setPosition(float turns) override {
    _sim.requestPositionTurns(turns);
    return true;
  }
  bool setTorque(float torqueNm) override {
    _sim.requestTorque(torqueNm);
    return true;
  }
  bool setControlMode(uint8_t controlMode, uint8_t inputMode) override {
    _controlMode = controlMode;
    _inputMode = inputMode;
    return true;
  }
  bool requestState(uint8_t state) override {
    _sim.requestState(state);
    return true;
  }
  bool enterClosedLoop() override {
    _sim.clearErrors();
    _sim.requestState(MOTOR_STATE_CLOSED_LOOP);
    return true;
  }
  bool enterIdle() override {
    _sim.requestState(MOTOR_STATE_IDLE);
    return true;
  }
  bool clearErrors() override {
    _sim.clearErrors();
    return true;
  }

  bool pollTelemetry(MotorTelemetry& outTelem) override {
    _sim.tick(millis());
    fill(outTelem);
    return true;
  }

  void updateTieredTelemetry(MotorTelemetry& ioTelem, uint32_t /*tickIndex*/) override {
    // The simulator has no serial budget: one refresh per tick is strictly better than the
    // real tiered schedule and never starves a field.
    _sim.tick(millis());
    fill(ioTelem);
  }

  bool readFeedbackFast(float& outTurns, float& outRpm) override {
    _sim.tick(millis());
    outTurns = _sim.actualRpm() / 60.0f * 0.0f;  // position not tracked externally here
    outRpm = _sim.actualRpm();
    return true;
  }

  bool readFloatProperty(const char* prop, float& outVal, uint32_t = 60) override {
    if (!prop) return false;
    if (strstr(prop, "vbus_voltage")) { outVal = _sim.vbus(); return true; }
    if (strstr(prop, "ibus")) { outVal = _sim.ibus(); return true; }
    if (strstr(prop, "vel_estimate")) { outVal = _sim.actualRpm() / 60.0f; return true; }
    if (strstr(prop, "torque_estimate")) { outVal = _sim.torque(); return true; }
    if (strstr(prop, "fet_thermistor")) { outVal = _sim.fetTemp(); return true; }
    if (strstr(prop, "Iq_measured")) { outVal = _sim.iq(); return true; }
    return false;
  }

  bool writeFloatProperty(const char* prop, float val) override {
    if (!prop) return false;
    if (strstr(prop, "vel_ramp_rate")) { _velRampRate = val; return true; }
    if (strstr(prop, "input_vel")) { _sim.requestVelocityRpm(val * 60.0f); return true; }
    return true;
  }

  bool sendRawCommand(const char* cmd, char* outBuf, size_t maxLen, uint32_t = 100) override {
    if (!cmd) return false;
    if (strcmp(cmd, "sc") == 0) return clearErrors();
    if (outBuf && maxLen > 0) {
      if (strstr(cmd, "vbus_voltage")) { snprintf(outBuf, maxLen, "%.2f", _sim.vbus()); return true; }
      if (strstr(cmd, "active_errors")) { snprintf(outBuf, maxLen, "%u", (unsigned)_sim.axisError()); return true; }
      if (strstr(cmd, "current_state")) { snprintf(outBuf, maxLen, "%u", (unsigned)_sim.state()); return true; }
      if (strstr(cmd, "control_mode")) { snprintf(outBuf, maxLen, "%u", (unsigned)_controlMode); return true; }
      if (strstr(cmd, "input_mode")) { snprintf(outBuf, maxLen, "%u", (unsigned)_inputMode); return true; }
      outBuf[0] = '\0';
    }
    return true;
  }

private:
  void fill(MotorTelemetry& t) {
    t.vbus_voltage = _sim.vbus();
    t.ibus_current = _sim.ibus();
    t.actual_rpm = _sim.actualRpm();
    t.power_watts = _sim.power();
    t.torque_estimate = _sim.torque();
    t.iq_measured = _sim.iq();
    t.fet_temperature = _sim.fetTemp();
    t.axis_error = _sim.axisError();
    t.disarm_reason = (!_sim.isArmed() && _sim.state() == MOTOR_STATE_IDLE) ? 1u : 0u;
    t.current_state = _sim.state();
    t.control_mode = _controlMode;
    t.input_mode = _inputMode;
    t.is_armed = _sim.isArmed();
  }

  MotorSim _sim;
  uint8_t _controlMode = 2;  // VELOCITY_CONTROL
  uint8_t _inputMode = 2;    // VEL_RAMP
  float _velRampRate = ODRIVE_VEL_RAMP_RATE;
  bool _specApplied = false;
};

VirtualMotorBus g_bus;

}  // namespace

IMotorBus& motorBus() { return g_bus; }
