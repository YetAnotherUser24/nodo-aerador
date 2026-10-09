#pragma once
/**
 * @file motor_bus.h
 * @brief Device-boundary interface for the aeration motor (ODrive S1 + BLDC paddle).
 *
 * This is the *only* seam between the node's control logic and the motor hardware.
 * The real build binds it to `ODriveUART`; the virtual build binds it to a simulated
 * motor + ODrive model. `main.cpp` never knows which one it holds.
 *
 * The surface is deliberately minimal: it exposes exactly what the node exercises.
 * Verified consumers:
 *   - control loop:      begin, clearErrors, setControlMode, writeFloatProperty,
 *                        enterClosedLoop, enterIdle, setVelocityRpm, pollTelemetry,
 *                        updateTieredTelemetry
 *   - telemetry / SD:    vbus, ibus, rpm, iq, torque, fet_temp, state, errors, armed
 */

#include <stdint.h>
#include <stddef.h>

/** Telemetry snapshot shared by both implementations (mirrors the ODrive fields). */
struct MotorTelemetry {
  float vbus_voltage = 0.0f;
  float ibus_current = 0.0f;
  float actual_rpm = 0.0f;
  float power_watts = 0.0f;
  float torque_estimate = 0.0f;  // Estimated mechanical torque (Nm)
  float iq_measured = 0.0f;      // q-axis phase current (A)
  float fet_temperature = 0.0f;  // Inverter MOSFET temperature (degC)
  uint32_t axis_error = 0;       // active_errors bitmask
  uint32_t disarm_reason = 0;    // disarm_reason bitmask
  uint8_t current_state = 0;     // 1 = IDLE, 8 = CLOSED_LOOP_CONTROL
  uint8_t control_mode = 0;      // 2 = VELOCITY_CONTROL
  uint8_t input_mode = 0;        // 2 = VEL_RAMP
  bool is_armed = false;
};

/** ODrive axis states we care about. */
enum : uint8_t {
  MOTOR_STATE_IDLE = 1,
  MOTOR_STATE_CLOSED_LOOP = 8,
};

class IMotorBus {
public:
  virtual ~IMotorBus() = default;

  /** Advance the device model / refresh cached hardware state. Call once per control tick. */
  virtual void tick(uint32_t nowMs) = 0;

  virtual bool begin() = 0;

  // --- Actuation -----------------------------------------------------------
  virtual bool setVelocityRpm(float rpm) = 0;
  virtual bool setPosition(float turns) = 0;
  virtual bool setTorque(float torqueNm) = 0;
  virtual bool setControlMode(uint8_t controlMode, uint8_t inputMode) = 0;
  virtual bool requestState(uint8_t state) = 0;
  virtual bool enterClosedLoop() = 0;
  virtual bool enterIdle() = 0;
  virtual bool clearErrors() = 0;

  // --- Telemetry -----------------------------------------------------------
  /** Blocking full read (used at boot and by bench tools). */
  virtual bool pollTelemetry(MotorTelemetry& outTelem) = 0;
  /** Rate-limited "<= N queries per tick" read, mirroring the real serial budget. */
  virtual void updateTieredTelemetry(MotorTelemetry& ioTelem, uint32_t tickIndex) = 0;
  /** Fast `f 0` feedback: position (turns) + velocity (RPM). */
  virtual bool readFeedbackFast(float& outTurns, float& outRpm) = 0;

  // --- Property / raw access (boot + bench) --------------------------------
  virtual bool readFloatProperty(const char* prop, float& outVal, uint32_t timeoutMs = 60) = 0;
  virtual bool writeFloatProperty(const char* prop, float val) = 0;
  virtual bool sendRawCommand(const char* cmd, char* outBuf = nullptr, size_t maxLen = 0,
                              uint32_t timeoutMs = 100) = 0;
};

/** Compile-time factory. Each build provides exactly one definition. */
IMotorBus& motorBus();
