/**
 * @file motor_bus_real.cpp
 * @brief Real `IMotorBus`: thin adapter over the existing ODriveUART driver.
 *
 * Build: the default (any non-virtual env). Contains no control logic — only the wiring
 * between the node's interface and the ODrive S1 ASCII driver.
 */

#include "motor_bus.h"
#include "odrive_uart.h"
#include "config.h"

#include <Arduino.h>

namespace {

class RealMotorBus : public IMotorBus {
public:
  RealMotorBus() : _serial(makeSerial()), _uart(_serial, ODRIVE_SERIAL_RX_PIN, ODRIVE_SERIAL_TX_PIN, ODRIVE_BAUD_RATE) {}

  void tick(uint32_t /*nowMs*/) override {
    // The real driver is ticked through updateTieredTelemetry(); nothing to do here.
  }

  bool begin() override { return _uart.begin(); }
  bool setVelocityRpm(float rpm) override { return _uart.setVelocityRpm(rpm); }
  bool setPosition(float turns) override { return _uart.setPosition(turns); }
  bool setTorque(float torqueNm) override { return _uart.setTorque(torqueNm); }
  bool setControlMode(uint8_t c, uint8_t i) override { return _uart.setControlMode(c, i); }
  bool requestState(uint8_t s) override { return _uart.requestState(s); }
  bool enterClosedLoop() override { return _uart.enterClosedLoop(); }
  bool enterIdle() override { return _uart.enterIdle(); }
  bool clearErrors() override { return _uart.clearErrors(); }

  bool pollTelemetry(MotorTelemetry& outTelem) override {
    return _uart.pollTelemetry(reinterpret_cast<ODriveTelemetry&>(outTelem));
  }

  void updateTieredTelemetry(MotorTelemetry& ioTelem, uint32_t tickIndex) override {
    _uart.updateTieredTelemetry(reinterpret_cast<ODriveTelemetry&>(ioTelem), tickIndex);
  }

  bool readFeedbackFast(float& outTurns, float& outRpm) override {
    return _uart.readFeedbackFast(outTurns, outRpm);
  }

  bool readFloatProperty(const char* p, float& v, uint32_t t = 60) override { return _uart.readFloatProperty(p, v, t); }
  bool writeFloatProperty(const char* p, float v) override { return _uart.writeFloatProperty(p, v); }
  bool sendRawCommand(const char* c, char* b, size_t n, uint32_t t = 100) override {
    return _uart.sendRawCommand(c, b, n, t);
  }

private:
  static HardwareSerial& makeSerial() {
#if CONFIG_IDF_TARGET_ESP32S3
    static HardwareSerial serial(1);
#else
    static HardwareSerial serial(2);
#endif
    return serial;
  }

  HardwareSerial& _serial;
  ODriveUART _uart;
};

RealMotorBus g_bus;

}  // namespace

IMotorBus& motorBus() { return g_bus; }
