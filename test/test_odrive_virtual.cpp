#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

// Standalone self-check test for ODriveVirtual physics, state machine, and telemetry calculations
// Compiles directly with standard C++ (g++, clang++, or platformio test)

struct ODriveTelemetry {
  float vbus_voltage = 0.0f;
  float ibus_current = 0.0f;
  float actual_rpm = 0.0f;
  float power_watts = 0.0f;
  uint32_t axis_error = 0;
  uint8_t current_state = 0;
  uint8_t control_mode = 0;
  uint8_t input_mode = 0;
  bool is_armed = false;
};

// Minimal standalone simulation harness matching ODriveVirtual logic
class TestODriveSimulator {
public:
  TestODriveSimulator(float nominalVbus = 24.0f, float rampRate = 1500.0f)
      : _nominalVbus(nominalVbus),
        _rampRate(rampRate),
        _targetRpm(0.0f),
        _actualRpm(0.0f),
        _vbus(nominalVbus),
        _ibus(0.15f),
        _power(0.0f),
        _state(1),
        _axisError(0),
        _isArmed(false),
        _simulatedTimeMs(0) {}

  void begin() {
    _state = 1;
    _isArmed = false;
    _axisError = 0;
    _targetRpm = 0.0f;
    _actualRpm = 0.0f;
    _vbus = _nominalVbus;
    _ibus = 0.15f;
    _power = _vbus * _ibus;
  }

  void enterClosedLoop() {
    _axisError = 0;
    _state = 8;
    _isArmed = true;
  }

  void enterIdle() {
    _state = 1;
    _isArmed = false;
    _targetRpm = 0.0f;
  }

  void setVelocityRpm(float rpm) {
    if (!_isArmed || _state != 8) {
      _targetRpm = 0.0f;
      return;
    }
    _targetRpm = fminf(fmaxf(rpm, 0.0f), 3500.0f);
  }

  void setSimulatedError(uint32_t err) {
    _axisError = err;
    if (err != 0) enterIdle();
  }

  void clearErrors() {
    _axisError = 0;
  }

  void stepSimulation(uint32_t dtMs) {
    _simulatedTimeMs += dtMs;
    float dt = (float)dtMs / 1000.0f;
    if (dt <= 0.0f) return;

    float prevRpm = _actualRpm;

    if (!_isArmed || _state != 8) {
      float coastRamp = _rampRate * 1.5f * dt;
      if (_actualRpm > coastRamp) _actualRpm -= coastRamp;
      else _actualRpm = 0.0f;
    } else {
      float maxDelta = _rampRate * dt;
      if (_targetRpm > _actualRpm) _actualRpm = fminf(_actualRpm + maxDelta, _targetRpm);
      else if (_targetRpm < _actualRpm) _actualRpm = fmaxf(_actualRpm - maxDelta, _targetRpm);
    }

    float normSpeed = _actualRpm / 3500.0f;
    float loadCurrent = 9.5f * (normSpeed * normSpeed);
    float dRpmDt = fabsf(_actualRpm - prevRpm) / dt;
    float accelCurrent = dRpmDt * 0.0015f;

    _ibus = 0.15f + loadCurrent + accelCurrent;
    _vbus = _nominalVbus - (_ibus * 0.035f);
    _power = _vbus * _ibus;
  }

  void pollTelemetry(ODriveTelemetry& out) {
    out.vbus_voltage = _vbus;
    out.ibus_current = _ibus;
    out.actual_rpm = _actualRpm;
    out.power_watts = _power;
    out.axis_error = _axisError;
    out.current_state = _state;
    out.is_armed = _isArmed;
  }

private:
  float _nominalVbus;
  float _rampRate;
  float _targetRpm;
  float _actualRpm;
  float _vbus;
  float _ibus;
  float _power;
  uint8_t _state;
  uint32_t _axisError;
  bool _isArmed;
  uint32_t _simulatedTimeMs;
};

int main() {
  printf("[TEST] Running ODrive Virtual Driver Self-Check...\n");

  TestODriveSimulator sim(24.0f, 1500.0f);
  ODriveTelemetry telem;

  // Test 1: Initialization state
  sim.begin();
  sim.pollTelemetry(telem);
  assert(telem.current_state == 1);
  assert(!telem.is_armed);
  assert(telem.actual_rpm == 0.0f);
  assert(fabsf(telem.vbus_voltage - 24.0f) < 0.05f);
  assert(telem.ibus_current < 0.2f);
  printf("  -> Test 1 (Initial Idle State): PASSED (State 1, Disarmed, 24V, 0 RPM)\n");

  // Test 2: Arming into Closed Loop
  sim.enterClosedLoop();
  sim.pollTelemetry(telem);
  assert(telem.current_state == 8);
  assert(telem.is_armed);
  printf("  -> Test 2 (Closed Loop State Transition): PASSED (State 8, Armed)\n");

  // Test 3: Velocity ramp acceleration dynamics (smooth transition, no instant jumping)
  sim.setVelocityRpm(3000.0f);
  // After 100ms (at 1500 RPM/s ramp rate), RPM should be around 150 RPM, NOT 3000 RPM!
  sim.stepSimulation(100);
  sim.pollTelemetry(telem);
  assert(telem.actual_rpm > 50.0f && telem.actual_rpm < 250.0f);
  assert(telem.actual_rpm < 3000.0f);
  printf("  -> Test 3a (Ramp In-Flight Check): PASSED (RPM at 100ms: %.1f RPM)\n", telem.actual_rpm);

  // Advance 2 seconds (total 2.1s): 1500 RPM/s * 2.1s = 3150 -> clamped to target 3000 RPM
  for (int i = 0; i < 40; i++) {
    sim.stepSimulation(50);
  }
  sim.pollTelemetry(telem);
  assert(fabsf(telem.actual_rpm - 3000.0f) < 1.0f);
  printf("  -> Test 3b (Steady State Target RPM Reached): PASSED (%.1f RPM)\n", telem.actual_rpm);

  // Test 4: Current and Power physics under aeration load
  assert(telem.ibus_current > 5.0f); // Hydrodynamic load at 3000 RPM consumes significant current
  assert(telem.power_watts > 100.0f); // Power = V * I > 100W
  assert(telem.vbus_voltage < 24.0f); // Voltage drops slightly under high current load
  printf("  -> Test 4 (Power & Load Current Physics): PASSED (I: %.2fA | V: %.2fV | P: %.1fW)\n",
         telem.ibus_current, telem.vbus_voltage, telem.power_watts);

  // Test 5: Deceleration & Emergency Stop / Disarm
  sim.enterIdle();
  sim.pollTelemetry(telem);
  assert(telem.current_state == 1);
  assert(!telem.is_armed);
  // After 1.5 seconds of coast-down
  for (int i = 0; i < 30; i++) {
    sim.stepSimulation(50);
  }
  sim.pollTelemetry(telem);
  assert(telem.actual_rpm == 0.0f);
  assert(telem.ibus_current < 0.2f);
  printf("  -> Test 5 (Disarm & Deceleration to 0 RPM): PASSED (State 1, 0 RPM, Idle Current)\n");

  // Test 6: Fault injection and error clearing
  sim.setSimulatedError(0x00000004); // Simulated overvoltage/motor error
  sim.pollTelemetry(telem);
  assert(telem.axis_error == 0x00000004);
  assert(telem.current_state == 1);
  sim.clearErrors();
  sim.pollTelemetry(telem);
  assert(telem.axis_error == 0);
  printf("  -> Test 6 (Fault Injection & Error Clear): PASSED (Error latched and cleared)\n");

  printf("[TEST] ALL VIRTUAL ODRIVE CHECKS PASSED SUCCESSFULLY.\n");
  return 0;
}
