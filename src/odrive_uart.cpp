#include "odrive_uart.h"
#include "config.h"

ODriveUART::ODriveUART(HardwareSerial& serialPort, uint8_t rxPin, uint8_t txPin, uint32_t baudRate)
    : _serial(serialPort), _rxPin(rxPin), _txPin(txPin), _baud(baudRate) {}

bool ODriveUART::begin() {
  _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
  delay(100);
  // Clear any existing errors on ODrive S1
  clearErrors();
  delay(50);
  // Ensure velocity limit allows up to 3500 RPM (58.3 turns/s)
  sendCommand("w axis0.controller.config.vel_limit 65.0");
  delay(30);
  // Configure Velocity Control with Velocity Ramp input mode
  sendCommand("w axis0.controller.config.control_mode 2");
  delay(30);
  sendCommand("w axis0.controller.config.input_mode 2");
  delay(30);
  // Set maximum velocity ramp rate (10 rev/s^2)
  char rampCmd[48];
  snprintf(rampCmd, sizeof(rampCmd), "w axis0.controller.config.vel_ramp_rate %.2f", ODRIVE_VEL_RAMP_RATE);
  sendCommand(rampCmd);
  return true;
}

bool ODriveUART::sendCommand(const char* cmd) {
  while (_serial.available()) _serial.read(); // flush RX
#if defined(ODRIVE_UART_DEBUG) && ODRIVE_UART_DEBUG
  Serial.printf("[ODriveUART] Sending cmd: %s\n", cmd);
#endif
  _serial.print(cmd);
  _serial.print('\n');
  return true;
}

bool ODriveUART::readFloatResponse(const char* queryCmd, float& outValue, uint32_t timeoutMs) {
  // Drain any lingering bytes or previous timeouts to prevent desynchronization
  while (_serial.available()) {
    _serial.read();
  }
  sendCommand(queryCmd);
  
  unsigned long start = millis();
  char buf[32];
  uint8_t idx = 0;
  
  while (millis() - start < timeoutMs) {
    if (_serial.available()) {
      char c = _serial.read();
      if (c == '\n' || c == '\r') {
        if (idx > 0) {
          buf[idx] = '\0';
          // Reject error responses like "invalid property"
          if (isdigit((unsigned char)buf[0]) || buf[0] == '-' || buf[0] == '+' || buf[0] == '.') {
            outValue = (float)atof(buf);
            return true;
          }
          return false;
        }
      } else if (idx < sizeof(buf) - 1) {
        buf[idx++] = c;
      }
    } else {
      delayMicroseconds(50);
    }
  }
  return false;
}

bool ODriveUART::readUintResponse(const char* queryCmd, uint32_t& outValue, uint32_t timeoutMs) {
  // Drain any lingering bytes or previous timeouts to prevent desynchronization
  while (_serial.available()) {
    _serial.read();
  }
  sendCommand(queryCmd);
  
  unsigned long start = millis();
  char buf[32];
  uint8_t idx = 0;
  
  while (millis() - start < timeoutMs) {
    if (_serial.available()) {
      char c = _serial.read();
      if (c == '\n' || c == '\r') {
        if (idx > 0) {
          buf[idx] = '\0';
          if (isdigit((unsigned char)buf[0])) {
            outValue = (uint32_t)strtoul(buf, NULL, 10);
            return true;
          }
          return false;
        }
      } else if (idx < sizeof(buf) - 1) {
        buf[idx++] = c;
      }
    } else {
      delayMicroseconds(50);
    }
  }
  return false;
}

bool ODriveUART::setVelocityRpm(float rpm) {
  // ODrive S1 expects velocity in turns/sec (rev/s)
  float turnsPerSec = rpm / 60.0f;
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "v 0 %.3f 0", turnsPerSec);
  return sendCommand(cmd);
}

bool ODriveUART::setPosition(float turns) {
  char cmd[48];
  snprintf(cmd, sizeof(cmd), "w axis0.controller.input_pos %.4f", turns);
  sendCommand(cmd);
  delay(10);
  snprintf(cmd, sizeof(cmd), "p 0 %.4f 0 0", turns);
  return sendCommand(cmd);
}

bool ODriveUART::setTorque(float torqueNm) {
  char cmd[48];
  snprintf(cmd, sizeof(cmd), "w axis0.controller.input_torque %.4f", torqueNm);
  sendCommand(cmd);
  delay(10);
  snprintf(cmd, sizeof(cmd), "c 0 %.4f", torqueNm);
  return sendCommand(cmd);
}

bool ODriveUART::setControlMode(uint8_t controlMode, uint8_t inputMode) {
  char cmd[64];
  // 1. Enter IDLE so controller allows changing control & input modes
  requestState(1);
  delay(30);

  // 2. Set control mode
  snprintf(cmd, sizeof(cmd), "w axis0.controller.config.control_mode %u", controlMode);
  sendCommand(cmd);
  delay(30);

  // 3. Set input mode
  snprintf(cmd, sizeof(cmd), "w axis0.controller.config.input_mode %u", inputMode);
  sendCommand(cmd);
  delay(30);

  // 4. Re-arm to CLOSED_LOOP_CONTROL (state 8)
  return requestState(8);
}

bool ODriveUART::requestState(uint8_t state) {
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "w axis0.requested_state %u", state);
  return sendCommand(cmd);
}

bool ODriveUART::enterClosedLoop() {
  // Clear any lingering faults and request state 8 (preserves current control mode)
  clearErrors();
  delay(30);
  return requestState(8);
}

bool ODriveUART::enterIdle() {
  // AXIS_STATE_IDLE = 1
  return requestState(1);
}

bool ODriveUART::clearErrors() {
  // ODrive clear errors ASCII command
  return sendCommand("sc");
}

bool ODriveUART::readFloatProperty(const char* prop, float& outVal, uint32_t timeoutMs) {
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "r %s", prop);
  return readFloatResponse(cmd, outVal, timeoutMs);
}

bool ODriveUART::writeFloatProperty(const char* prop, float val) {
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "w %s %.4f", prop, val);
  return sendCommand(cmd);
}

bool ODriveUART::readFeedbackFast(float& outTurns, float& outRpm) {
  char buf[32];
  if (sendRawCommand("f 0", buf, sizeof(buf), 12)) {
    float pos = 0.0f;
    float vel = 0.0f;
    if (sscanf(buf, "%f %f", &pos, &vel) == 2) {
      outTurns = pos;
      outRpm = vel * 60.0f;
      return true;
    }
  }
  return false;
}

void ODriveUART::updateTieredTelemetry(ODriveTelemetry& ioTelem, uint32_t tickIndex) {
  // Even ticks (0, 2, 4, 6... @ 10 Hz): High-rate dynamic feedback (vel + torque)
  if ((tickIndex % 2) == 0) {
    float velTurnsSec = 0.0f;
    if (readFloatResponse("r axis0.vel_estimate", velTurnsSec, 15)) {
      ioTelem.actual_rpm = velTurnsSec * 60.0f;
    }
    float iq = 0.0f;
    if (readFloatResponse("r axis0.motor.foc.Iq_measured", iq, 12)) {
      ioTelem.iq_measured = iq;
    }
    float torque = 0.0f;
    if (readFloatResponse("r axis0.motor.torque_estimate", torque, 12)) {
      ioTelem.torque_estimate = torque;
    }
    return;
  }

  // Odd ticks: Interleave medium and slow tier queries (strictly 1 query per tick)
  uint32_t slot = (tickIndex / 2) % 10;
  switch (slot) {
    case 0:
    case 5: {
      float ibus = 0.0f;
      if (readFloatResponse("r ibus", ibus, 15)) {
        ioTelem.ibus_current = ibus;
        ioTelem.power_watts = ioTelem.vbus_voltage * ibus;
      }
      break;
    }
    case 1:
    case 6: {
      float vbus = 0.0f;
      if (readFloatResponse("r vbus_voltage", vbus, 15)) {
        ioTelem.vbus_voltage = vbus;
      }
      break;
    }
    case 2:
    case 9: {
      uint32_t state = 0;
      if (readUintResponse("r axis0.current_state", state, 15)) {
        ioTelem.current_state = (uint8_t)state;
        ioTelem.is_armed = (state == 8);
      }
      break;
    }
    case 3:
    case 8: {
      float temp = 0.0f;
      if (readFloatResponse("r axis0.fet_thermistor.temperature", temp, 15) ||
          readFloatResponse("r axis0.motor.fet_thermistor.temperature", temp, 15)) {
        ioTelem.fet_temperature = temp;
      }
      break;
    }
    case 4: {
      uint32_t activeErrors = 0;
      if (readUintResponse("r axis0.active_errors", activeErrors, 15)) {
        ioTelem.axis_error = activeErrors;
      }
      break;
    }
    case 7: {
      uint32_t disarm = 0;
      if (readUintResponse("r axis0.disarm_reason", disarm, 15)) {
        ioTelem.disarm_reason = disarm;
      }
      break;
    }
    default:
      break;
  }
}

bool ODriveUART::pollTelemetry(ODriveTelemetry& outTelem) {
  float vbus = 0.0f;
  float ibus = 0.0f;
  float velTurnsSec = 0.0f;
  float torque = 0.0f;
  uint32_t state = 0;
  bool ok = false;

  if (readFloatResponse("r vbus_voltage", vbus, 30)) {
    outTelem.vbus_voltage = vbus;
    ok = true;
  }
  
  if (readFloatResponse("r ibus", ibus, 30)) {
    outTelem.ibus_current = ibus;
    outTelem.power_watts = outTelem.vbus_voltage * ibus;
    ok = true;
  }

  if (readFloatResponse("r axis0.vel_estimate", velTurnsSec, 30)) {
    outTelem.actual_rpm = velTurnsSec * 60.0f;
    ok = true;
  }

  float iq = 0.0f;
  if (readFloatResponse("r axis0.motor.foc.Iq_measured", iq, 20)) {
    outTelem.iq_measured = iq;
    ok = true;
  }

  if (readFloatResponse("r axis0.motor.torque_estimate", torque, 20)) {
    outTelem.torque_estimate = torque;
    ok = true;
  }

  if (readUintResponse("r axis0.current_state", state, 30)) {
    outTelem.current_state = (uint8_t)state;
    outTelem.is_armed = (state == 8);
    ok = true;
  }

  uint32_t activeErrors = 0;
  if (readUintResponse("r axis0.active_errors", activeErrors, 30)) {
    outTelem.axis_error = activeErrors;
    ok = true;
  }

  return ok;
}

bool ODriveUART::sendRawCommand(const char* cmd, char* outBuf, size_t maxLen, uint32_t timeoutMs) {
  while (_serial.available()) {
    _serial.read();
  }
  sendCommand(cmd);
  if (!outBuf || maxLen == 0) return true;

  outBuf[0] = '\0';
  unsigned long start = millis();
  size_t idx = 0;

  while (millis() - start < timeoutMs) {
    if (_serial.available()) {
      char c = _serial.read();
      if (c == '\n' || c == '\r') {
        if (idx > 0) {
          outBuf[idx] = '\0';
          return true;
        }
      } else if (idx < maxLen - 1) {
        outBuf[idx++] = c;
      }
    } else {
      delayMicroseconds(50);
    }
  }
  outBuf[idx] = '\0';
  return (idx > 0);
}
