#pragma once
#include <Arduino.h>

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
  bool readFloatProperty(const char* prop, float& outVal, uint32_t timeoutMs = 60);
  bool writeFloatProperty(const char* prop, float val);
  bool sendRawCommand(const char* cmd, char* outBuf = nullptr, size_t maxLen = 0, uint32_t timeoutMs = 100);

private:
  HardwareSerial& _serial;
  uint8_t _rxPin;
  uint8_t _txPin;
  uint32_t _baud;
  
  bool sendCommand(const char* cmd);
  bool readFloatResponse(const char* queryCmd, float& outValue, uint32_t timeoutMs = 50);
  bool readUintResponse(const char* queryCmd, uint32_t& outValue, uint32_t timeoutMs = 50);
};
