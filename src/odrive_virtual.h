#pragma once
#include <Arduino.h>
#include "odrive_uart.h"

/**
 * @brief Virtual ODrive S1 Driver for AquaControl
 * 
 * Drop-in simulation replacement for ODriveUART. Simulates realistic motor
 * dynamics (rotational inertia, ramp acceleration/deceleration, current draw
 * under aeration load, power consumption, DC bus voltage drop) and state
 * machine transitions without physical ODrive hardware.
 */
class ODriveVirtual {
public:
  explicit ODriveVirtual(float nominalVbus = 24.0f, float rampRateRpmPerSec = 1500.0f);

  bool begin();
  bool setVelocityRpm(float rpm);
  bool setPosition(float turns);
  bool setTorque(float torqueNm);
  bool setControlMode(uint8_t controlMode, uint8_t inputMode);
  bool requestState(uint8_t state);
  bool enterClosedLoop();
  bool enterIdle();
  bool clearErrors();
  bool pollTelemetry(ODriveTelemetry& outTelem);
  void updateTieredTelemetry(ODriveTelemetry& ioTelem, uint32_t tickIndex) { pollTelemetry(ioTelem); }
  bool readFloatProperty(const char* prop, float& outVal, uint32_t timeoutMs = 60);
  bool writeFloatProperty(const char* prop, float val);
  bool sendRawCommand(const char* cmd, char* outBuf = nullptr, size_t maxLen = 0, uint32_t timeoutMs = 100);

  // Simulation controls / test inspection
  void setSimulatedError(uint32_t errorCode);
  void setNominalVbus(float vbus);
  void setRampRate(float rpmPerSec);
  float getTargetRpm() const { return _targetRpm; }
  float getActualRpm() const { return _actualRpm; }

private:
  float _nominalVbus;
  float _rampRate;
  float _targetRpm;
  float _actualRpm;
  float _vbus;
  float _ibus;
  float _power;
  uint8_t _state;        // 1 = AXIS_STATE_IDLE, 8 = AXIS_STATE_CLOSED_LOOP_CONTROL
  uint8_t _controlMode;  // 2 = VELOCITY_CONTROL
  uint8_t _inputMode;    // 2 = VEL_RAMP
  uint32_t _axisError;
  bool _isArmed;
  unsigned long _lastUpdateMs;

  void updatePhysics(unsigned long nowMs);
};
