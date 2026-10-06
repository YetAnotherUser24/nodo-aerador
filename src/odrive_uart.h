#pragma once
#include <Arduino.h>

struct ODriveTelemetry {
  float vbus_voltage = 0.0f;
  float ibus_current = 0.0f;
  float actual_rpm = 0.0f;
  float power_watts = 0.0f;
  float torque_estimate = 0.0f;    // Estimated mechanical torque (Nm)
  float iq_measured = 0.0f;        // Phase current (A)
  float fet_temperature = 0.0f;    // Inverter MOSFET thermistor temperature (°C)
  uint32_t axis_error = 0;         // active_errors bitmask
  uint32_t disarm_reason = 0;      // disarm_reason bitmask
  uint8_t current_state = 0;
  uint8_t control_mode = 0;
  uint8_t input_mode = 0;
  bool is_armed = false;
};

class ODriveUART {
public:
  ODriveUART(HardwareSerial& serialPort, uint8_t rxPin, uint8_t txPin, uint32_t baudRate);
  
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
  void updateTieredTelemetry(ODriveTelemetry& ioTelem, uint32_t tickIndex);
  bool readFeedbackFast(float& outTurns, float& outRpm);
  bool readFloatProperty(const char* prop, float& outVal, uint32_t timeoutMs = 60);
  bool writeFloatProperty(const char* prop, float val);
  bool sendRawCommand(const char* cmd, char* outBuf = nullptr, size_t maxLen = 0, uint32_t timeoutMs = 100);

private:
  HardwareSerial& _serial;
  uint8_t _rxPin;
  uint8_t _txPin;
  uint32_t _baud;
  
  bool sendCommand(const char* cmd);
  bool readFloatResponse(const char* queryCmd, float& outValue, uint32_t timeoutMs = 12);
  bool readUintResponse(const char* queryCmd, uint32_t& outValue, uint32_t timeoutMs = 12);
};
