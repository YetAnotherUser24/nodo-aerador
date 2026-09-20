/**
 * @file test_odrive_uart.cpp
 * @brief Advanced ODrive UART Engineering Console & Parameter Tuner
 *
 * Hardware Configuration:
 *   - ESP32 UART2 (Serial2)
 *   - Default RX Pin: GPIO 16 (connects to ODrive S1 TX)
 *   - Default TX Pin: GPIO 17 (connects to ODrive S1 RX)
 *   - Common GND (MANDATORY)
 *   - Default Baud: 115200
 *
 * Console Interface (USB Serial @ 115200 baud):
 *   Type 'h' or '?' in the monitor for the full categorized command menu.
 */

#include <Arduino.h>
#include "odrive_uart.h"

static uint8_t currentRxPin = 16;
static uint8_t currentTxPin = 17;
static uint32_t currentBaud = 115200;

static HardwareSerial ODriveSerial(2);
static ODriveUART odrive(ODriveSerial, currentRxPin, currentTxPin, currentBaud);

static bool streamTelemetry = false;
static unsigned long lastStreamMs = 0;
static const unsigned long STREAM_INTERVAL_MS = 500;

static char inputBuffer[128];
static size_t inputIdx = 0;

static const char* stateToString(uint8_t state) {
  switch (state) {
    case 0: return "UNDEFINED (No comms / uncalibrated)";
    case 1: return "IDLE (Disarmed)";
    case 2: return "STARTUP_SEQUENCE";
    case 3: return "FULL_CALIBRATION_SEQUENCE";
    case 4: return "MOTOR_CALIBRATION";
    case 6: return "ENCODER_INDEX_SEARCH";
    case 7: return "ENCODER_OFFSET_CALIBRATION";
    case 8: return "CLOSED_LOOP_CONTROL (Armed & Active)";
    case 9: return "LOCKIN_SPIN";
    case 10: return "ENCODER_DIR_FIND";
    case 11: return "HOMING";
    default: return "UNKNOWN";
  }
}

static const char* controlModeToString(uint8_t mode) {
  switch (mode) {
    case 0: return "VOLTAGE_CONTROL";
    case 1: return "TORQUE_CONTROL";
    case 2: return "VELOCITY_CONTROL";
    case 3: return "POSITION_CONTROL";
    default: return "UNKNOWN";
  }
}

static const char* inputModeToString(uint8_t mode) {
  switch (mode) {
    case 0: return "INACTIVE";
    case 1: return "PASSTHROUGH";
    case 2: return "VEL_RAMP";
    case 3: return "POS_FILTER";
    case 4: return "MIX_CHANNELS";
    case 5: return "TRAP_TRAJ";
    case 6: return "TORQUE_RAMP";
    case 7: return "MIRROR";
    case 8: return "TUNING";
    default: return "UNKNOWN";
  }
}

static void printMenu() {
  Serial.println();
  Serial.println("=========================================================================");
  Serial.println("                  ODrive S1 Advanced Engineering Console                 ");
  Serial.printf ("       UART: Serial2 | RX: Pin %d | TX: Pin %d | Baud: %u\n", currentRxPin, currentTxPin, currentBaud);
  Serial.println("=========================================================================");
  Serial.println(" [1] Motion & State Control:");
  Serial.println("       arm / c               -> Arm motor (Enter CLOSED_LOOP_CONTROL)");
  Serial.println("       disarm / i            -> Disarm motor (Enter IDLE)");
  Serial.println("       s                     -> Emergency Stop (Zero setpoints + disarm)");
  Serial.println("       cal                   -> Run Full Motor & Encoder Calibration");
  Serial.println("       e                     -> Clear all latched ODrive errors ('sc')");
  Serial.println();
  Serial.println(" [2] Control Mode Switching:");
  Serial.println("       mode vel [ramp|pass]  -> Switch to Velocity Control (Ramped or Passthrough)");
  Serial.println("       mode pos [filter|trap|pass] -> Switch to Position Control");
  Serial.println("       mode trq              -> Switch to Torque Control");
  Serial.println();
  Serial.println(" [3] Setpoints:");
  Serial.println("       v <rpm>               -> Velocity setpoint in RPM (e.g. 'v 300', 'v -150')");
  Serial.println("       vt <turns/s>          -> Velocity setpoint in turns/sec (e.g. 'vt 5.0')");
  Serial.println("       p <turns>             -> Position setpoint in turns (e.g. 'p 10.0', 'p 0')");
  Serial.println("       trq <Nm>              -> Torque setpoint in Nm (e.g. 'trq 0.15')");
  Serial.println();
  Serial.println(" [4] Parameter Tuning & Dashboard:");
  Serial.println("       params                -> Display full live parameters & gains dashboard");
  Serial.println("       vlim <turns/s>        -> Set velocity limit (e.g. 'vlim 20.0')");
  Serial.println("       ramp <turns/s^2>      -> Set velocity ramp rate (e.g. 'ramp 10.0')");
  Serial.println("       gain pos <val>        -> Tune position loop gain (pos_gain)");
  Serial.println("       gain vel <val>        -> Tune velocity loop gain (vel_gain)");
  Serial.println("       gain vi <val>         -> Tune velocity integrator gain (vel_integrator_gain)");
  Serial.println("       ilim <amps>           -> Set motor current limit (current_soft_max)");
  Serial.println();
  Serial.println(" [5] Flash & Diagnostics:");
  Serial.println("       save                  -> Save configuration to ODrive NVS flash ('ss')");
  Serial.println("       reboot                -> Reboot ODrive controller ('sr')");
  Serial.println("       errors                -> Read active errors & disarm reasons");
  Serial.println("       t                     -> Print single telemetry snapshot");
  Serial.println("       m                     -> Toggle continuous 500ms telemetry monitor");
  Serial.println("       diag                  -> Run physical UART sniffer & checklist");
  Serial.println("       swap                  -> Swap RX & TX pins (16 <-> 17)");
  Serial.println("       baud <rate>           -> Change UART baud rate (e.g. 'baud 115200')");
  Serial.println("       r <property>          -> Direct read (e.g. 'r vbus_voltage')");
  Serial.println("       w <property> <val>    -> Direct write (e.g. 'w axis0.requested_state 8')");
  Serial.println("       h / ?                 -> Display this menu");
  Serial.println("=========================================================================\n");
}

static void printParametersDashboard() {
  Serial.println("\n==========================================================");
  Serial.println("                 ODrive Parameters Dashboard              ");
  Serial.println("==========================================================");

  float val = 0.0f;
  char buf[32] = {0};

  // 1. Control & Input Mode
  if (odrive.sendRawCommand("r axis0.controller.config.control_mode", buf, sizeof(buf), 60)) {
    uint8_t cMode = (uint8_t)atoi(buf);
    Serial.printf("  Control Mode        : %u (%s)\n", cMode, controlModeToString(cMode));
  } else {
    Serial.println("  Control Mode        : (Query timed out)");
  }

  if (odrive.sendRawCommand("r axis0.controller.config.input_mode", buf, sizeof(buf), 60)) {
    uint8_t iMode = (uint8_t)atoi(buf);
    Serial.printf("  Input Mode          : %u (%s)\n", iMode, inputModeToString(iMode));
  } else {
    Serial.println("  Input Mode          : (Query timed out)");
  }

  // 2. Limits
  if (odrive.readFloatProperty("axis0.controller.config.vel_limit", val, 60)) {
    Serial.printf("  Velocity Limit      : %.2f turns/s (%.1f RPM)\n", val, val * 60.0f);
  }
  if (odrive.readFloatProperty("axis0.controller.config.vel_ramp_rate", val, 60)) {
    Serial.printf("  Velocity Ramp Rate  : %.2f turns/s^2\n", val);
  }
  if (odrive.readFloatProperty("axis0.config.motor.current_soft_max", val, 60)) {
    Serial.printf("  Current Limit       : %.2f A\n", val);
  } else if (odrive.readFloatProperty("axis0.motor.config.current_lim", val, 60)) {
    Serial.printf("  Current Limit       : %.2f A\n", val);
  }

  Serial.println("----------------------------------------------------------");
  Serial.println("  Controller PID Gains:");
  if (odrive.readFloatProperty("axis0.controller.config.pos_gain", val, 60)) {
    Serial.printf("    pos_gain            : %.4f\n", val);
  }
  if (odrive.readFloatProperty("axis0.controller.config.vel_gain", val, 60)) {
    Serial.printf("    vel_gain            : %.6f\n", val);
  }
  if (odrive.readFloatProperty("axis0.controller.config.vel_integrator_gain", val, 60)) {
    Serial.printf("    vel_integrator_gain : %.6f\n", val);
  }

  Serial.println("----------------------------------------------------------");
  Serial.println("  Live Position & Velocity Estimates:");
  if (odrive.readFloatProperty("axis0.pos_estimate", val, 60)) {
    Serial.printf("    Actual Position     : %.3f turns\n", val);
  }
  if (odrive.readFloatProperty("axis0.vel_estimate", val, 60)) {
    Serial.printf("    Actual Velocity     : %.2f turns/s (%.1f RPM)\n", val, val * 60.0f);
  }
  Serial.println("==========================================================\n");
}

static void printErrorsDetailed() {
  Serial.println("\n--- [ODrive Error Inspection] ---");
  char buf[64] = {0};

  if (odrive.sendRawCommand("r axis0.active_errors", buf, sizeof(buf), 80)) {
    uint32_t activeErr = (uint32_t)strtoul(buf, NULL, 10);
    Serial.printf("  Active Errors : 0x%08X (%u)\n", activeErr, activeErr);
  } else {
    Serial.println("  Active Errors : (No response)");
  }

  if (odrive.sendRawCommand("r axis0.disarm_reason", buf, sizeof(buf), 80)) {
    uint32_t disarm = (uint32_t)strtoul(buf, NULL, 10);
    Serial.printf("  Disarm Reason : 0x%08X (%u)\n", disarm, disarm);
  }

  if (odrive.sendRawCommand("r axis0.procedure_result", buf, sizeof(buf), 80)) {
    uint32_t proc = (uint32_t)strtoul(buf, NULL, 10);
    Serial.printf("  Procedure Result: %u\n", proc);
  }
  Serial.println("---------------------------------\n");
}

static void runDiagnostics() {
  Serial.println("\n--- [Running Hardware UART Diagnostic] ---");
  Serial.printf("Testing Serial2: ESP32 Pin %d (RX) <---> ESP32 Pin %d (TX) @ %u baud\n",
                currentRxPin, currentTxPin, currentBaud);

  while (ODriveSerial.available()) ODriveSerial.read();

  Serial.println("[DIAG TX] Sending query: 'r vbus_voltage\\n'...");
  ODriveSerial.print("r vbus_voltage\n");

  unsigned long start = millis();
  int bytesReceived = 0;
  char lineBuf[64];
  size_t lineLen = 0;

  while (millis() - start < 600) {
    while (ODriveSerial.available()) {
      char c = ODriveSerial.read();
      bytesReceived++;
      if (lineLen < sizeof(lineBuf) - 1 && c != '\r' && c != '\n') {
        lineBuf[lineLen++] = c;
      }
      Serial.printf("  [RX Byte %2d] 0x%02X  ('%c')\n", bytesReceived, (uint8_t)c, (c >= 32 && c <= 126) ? c : '.');
    }
  }
  lineBuf[lineLen] = '\0';

  Serial.println("------------------------------------------");
  if (bytesReceived > 0) {
    Serial.printf("[DIAG RESULT] SUCCESS! Received %d bytes from ODrive.\n", bytesReceived);
    if (lineLen > 0) {
      Serial.printf("[DIAG RESULT] Parsed line: \"%s\" (Float value: %.2f V)\n", lineBuf, atof(lineBuf));
    }
  } else {
    Serial.println("[DIAG RESULT] FAILED: Zero bytes received in 600ms.");
    Serial.println("\n>>> HARDWARE TROUBLESHOOTING CHECKLIST <<<");
    Serial.printf("1. Check RX/TX Crossover:\n");
    Serial.printf("   - ESP32 Pin %d (RX) MUST connect to ODrive TX\n", currentRxPin);
    Serial.printf("   - ESP32 Pin %d (TX) MUST connect to ODrive RX\n", currentTxPin);
    Serial.println("   * TIP: Type 'swap' in this console to flip RX/TX pins without rewiring!");
    Serial.println("2. Check Ground:\n");
    Serial.println("   - Common GND wire MUST connect ESP32 GND to ODrive GND/ISO GND.");
    Serial.println("3. Check ODrive UART Enable:\n");
    Serial.println("   - In odrivetool: odrv0.config.enable_uart_a = True; odrv0.save_configuration()");
    Serial.println("4. Check Power:\n");
    Serial.println("   - Verify DC bus voltage is connected to ODrive and green power LED is ON.");
  }
  Serial.println("------------------------------------------\n");
}

static void printTelemetrySnapshot() {
  ODriveTelemetry telem;
  bool ok = odrive.pollTelemetry(telem);

  if (ok) {
    Serial.println("--- [ODrive Telemetry Snapshot] ---");
    Serial.printf("  Vbus Voltage : %.2f V\n", telem.vbus_voltage);
    Serial.printf("  Ibus Current : %.3f A\n", telem.ibus_current);
    Serial.printf("  Input Power  : %.2f W\n", telem.power_watts);
    Serial.printf("  Actual RPM   : %.1f RPM (%.2f rev/s)\n", telem.actual_rpm, telem.actual_rpm / 60.0f);
    Serial.printf("  Current State: %u (%s)\n", telem.current_state, stateToString(telem.current_state));
    Serial.printf("  Armed Status : %s\n", telem.is_armed ? "ARMED (Closed-Loop)" : "DISARMED (Idle/Error)");
    Serial.printf("  Control Mode : %u (%s)\n", telem.control_mode, controlModeToString(telem.control_mode));
    Serial.printf("  Input Mode   : %u (%s)\n", telem.input_mode, inputModeToString(telem.input_mode));
    Serial.printf("  Active Errors: 0x%08X (%u)\n", telem.axis_error, telem.axis_error);
    Serial.println("-----------------------------------");
  } else {
    Serial.println("\n[ERROR] ODrive UART communication timed out (no response received)!");
    Serial.println("  -> Type 'diag' to inspect the physical UART signal.");
    Serial.println("  -> Type 'swap' to test flipping RX & TX pins (16 <-> 17).\n");
  }
}

static void handleCommand(const char* line) {
  while (*line == ' ') line++;
  if (*line == '\0') return;

  // Help
  if (strcmp(line, "h") == 0 || strcmp(line, "?") == 0) {
    printMenu();
  }
  // Motion & State Control
  else if (strcmp(line, "arm") == 0 || strcmp(line, "c") == 0) {
    Serial.println("[CMD] Requesting CLOSED_LOOP_CONTROL (state 8)...");
    odrive.requestState(8);
    delay(100);
    printTelemetrySnapshot();
  }
  else if (strcmp(line, "disarm") == 0 || strcmp(line, "i") == 0) {
    Serial.println("[CMD] Requesting IDLE (state 1)...");
    odrive.enterIdle();
    delay(100);
    printTelemetrySnapshot();
  }
  else if (strcmp(line, "s") == 0) {
    Serial.println("[CMD] EMERGENCY STOP: Stopping motor & disarming...");
    odrive.setVelocityRpm(0.0f);
    odrive.setTorque(0.0f);
    odrive.enterIdle();
    delay(50);
    printTelemetrySnapshot();
  }
  else if (strcmp(line, "cal") == 0) {
    Serial.println("[CMD] Initiating FULL_CALIBRATION_SEQUENCE (state 3)...");
    Serial.println("  >> Motor will perform beep/resistance calibration and encoder search <<");
    odrive.requestState(3);
    delay(200);
    printTelemetrySnapshot();
  }
  else if (strcmp(line, "e") == 0) {
    Serial.println("[CMD] Clearing ODrive Errors ('sc')...");
    odrive.clearErrors();
    delay(100);
    printTelemetrySnapshot();
  }
  // Control Mode Switching: mode vel [ramp|pass] / mode pos [filter|trap|pass] / mode trq
  else if (strncmp(line, "mode ", 5) == 0) {
    const char* sub = line + 5;
    while (*sub == ' ') sub++;

    if (strncmp(sub, "vel", 3) == 0) {
      uint8_t inputMode = 1; // default passthrough
      if (strstr(sub, "ramp")) inputMode = 2; // VEL_RAMP
      Serial.printf("[CMD] Switching to VELOCITY CONTROL (control_mode=2, input_mode=%u - %s)...\n",
                    inputMode, (inputMode == 2) ? "VEL_RAMP" : "PASSTHROUGH");
      odrive.setControlMode(2, inputMode);
      delay(50);
      odrive.setVelocityRpm(0.0f);
      delay(50);
      printTelemetrySnapshot();
    }
    else if (strncmp(sub, "pos", 3) == 0) {
      uint8_t inputMode = 1; // default passthrough
      if (strstr(sub, "filter")) inputMode = 3; // POS_FILTER
      else if (strstr(sub, "trap")) inputMode = 5; // TRAP_TRAJ
      Serial.printf("[CMD] Switching to POSITION CONTROL (control_mode=3, input_mode=%u - %s)...\n",
                    inputMode, inputModeToString(inputMode));
      // Read current pos estimate and hold it during transition
      float curPos = 0.0f;
      odrive.readFloatProperty("axis0.pos_estimate", curPos, 60);
      odrive.setControlMode(3, inputMode);
      delay(50);
      odrive.setPosition(curPos);
      delay(50);
      printTelemetrySnapshot();
    }
    else if (strncmp(sub, "trq", 3) == 0) {
      Serial.println("[CMD] Switching to TORQUE CONTROL (control_mode=1, input_mode=1)...");
      odrive.setControlMode(1, 1);
      delay(50);
      odrive.setTorque(0.0f);
      delay(50);
      printTelemetrySnapshot();
    }
    else {
      Serial.println("[USAGE] mode vel [ramp|pass] | mode pos [filter|trap|pass] | mode trq");
    }
  }
  // Velocity Setpoints: v <rpm> or vel <rpm> or vt <turns/s>
  else if ((line[0] == 'v' && (line[1] == ' ' || line[1] == '\0')) || strncmp(line, "vel ", 4) == 0) {
    const char* arg = (line[0] == 'v' && line[1] != 'e') ? (line + 1) : (line + 4);
    while (*arg == ' ') arg++;
    if (*arg == '\0') {
      Serial.println("[USAGE] v <rpm> -> e.g. 'v 300' or 'v -150'");
      return;
    }
    float rpm = (float)atof(arg);

    // Auto-switch to Velocity Control (mode 2) if not already active
    ODriveTelemetry telem;
    if (odrive.pollTelemetry(telem)) {
      if (telem.control_mode != 2) {
        Serial.println("[AUTO-MODE] ODrive is not in velocity mode. Switching to VELOCITY CONTROL (mode 2)...");
        odrive.setControlMode(2, 1);
        delay(50);
      }
      if (!telem.is_armed) {
        odrive.requestState(8);
        delay(50);
      }
    }

    Serial.printf("[CMD] Setting velocity to %.1f RPM (%.2f rev/s)...\n", rpm, rpm / 60.0f);
    odrive.setVelocityRpm(rpm);
  }
  else if (strncmp(line, "vt ", 3) == 0) {
    const char* arg = line + 3;
    while (*arg == ' ') arg++;
    float turnsSec = (float)atof(arg);

    ODriveTelemetry telem;
    if (odrive.pollTelemetry(telem)) {
      if (telem.control_mode != 2) {
        Serial.println("[AUTO-MODE] Switching to VELOCITY CONTROL (mode 2)...");
        odrive.setControlMode(2, 1);
        delay(50);
      }
      if (!telem.is_armed) {
        odrive.requestState(8);
        delay(50);
      }
    }

    Serial.printf("[CMD] Setting velocity to %.2f turns/s (%.1f RPM)...\n", turnsSec, turnsSec * 60.0f);
    odrive.setVelocityRpm(turnsSec * 60.0f);
  }
  // Position Setpoints: p <turns> or pos <turns>
  else if ((line[0] == 'p' && (line[1] == ' ' || line[1] == '\0')) || strncmp(line, "pos ", 4) == 0) {
    const char* arg = (line[0] == 'p' && line[1] != 'o') ? (line + 1) : (line + 4);
    while (*arg == ' ') arg++;
    if (*arg == '\0') {
      Serial.println("[USAGE] p <turns> -> e.g. 'p 10.0' or 'p 0'");
      return;
    }
    float turns = (float)atof(arg);

    // Auto-switch to Position Control (mode 3) if not already active
    ODriveTelemetry telem;
    if (odrive.pollTelemetry(telem)) {
      if (telem.control_mode != 3) {
        Serial.println("[AUTO-MODE] ODrive is not in position mode. Switching to POSITION CONTROL (mode 3)...");
        odrive.setControlMode(3, 1);
        delay(50);
      }
      if (!telem.is_armed) {
        odrive.requestState(8);
        delay(50);
      }
    }

    Serial.printf("[CMD] Setting position setpoint to %.3f turns...\n", turns);
    odrive.setPosition(turns);
  }
  // Torque Setpoints: trq <Nm> or tq <Nm>
  else if (strncmp(line, "trq ", 4) == 0 || strncmp(line, "tq ", 3) == 0) {
    const char* arg = (line[1] == 'r') ? (line + 4) : (line + 3);
    while (*arg == ' ') arg++;
    float nm = (float)atof(arg);

    // Auto-switch to Torque Control (mode 1) if not already active
    ODriveTelemetry telem;
    if (odrive.pollTelemetry(telem)) {
      if (telem.control_mode != 1) {
        Serial.println("[AUTO-MODE] ODrive is not in torque mode. Switching to TORQUE CONTROL (mode 1)...");
        odrive.setControlMode(1, 1);
        delay(50);
      }
      if (!telem.is_armed) {
        odrive.requestState(8);
        delay(50);
      }
    }

    Serial.printf("[CMD] Setting torque setpoint to %.4f Nm...\n", nm);
    odrive.setTorque(nm);
  }
  // Parameters & Dashboard
  else if (strcmp(line, "params") == 0) {
    printParametersDashboard();
  }
  else if (strncmp(line, "vlim ", 5) == 0) {
    const char* arg = line + 5;
    while (*arg == ' ') arg++;
    float limit = (float)atof(arg);
    Serial.printf("[CMD] Setting velocity limit to %.2f turns/s (%.1f RPM)...\n", limit, limit * 60.0f);
    odrive.writeFloatProperty("axis0.controller.config.vel_limit", limit);
    delay(50);
    printParametersDashboard();
  }
  else if (strncmp(line, "ramp ", 5) == 0) {
    const char* arg = line + 5;
    while (*arg == ' ') arg++;
    float rampRate = (float)atof(arg);
    Serial.printf("[CMD] Setting velocity ramp rate to %.2f turns/s^2...\n", rampRate);
    odrive.writeFloatProperty("axis0.controller.config.vel_ramp_rate", rampRate);
    delay(50);
    printParametersDashboard();
  }
  else if (strncmp(line, "gain ", 5) == 0) {
    const char* arg = line + 5;
    while (*arg == ' ') arg++;
    if (strncmp(arg, "pos ", 4) == 0) {
      float g = (float)atof(arg + 4);
      Serial.printf("[CMD] Setting pos_gain to %.4f...\n", g);
      odrive.writeFloatProperty("axis0.controller.config.pos_gain", g);
    }
    else if (strncmp(arg, "vel ", 4) == 0) {
      float g = (float)atof(arg + 4);
      Serial.printf("[CMD] Setting vel_gain to %.6f...\n", g);
      odrive.writeFloatProperty("axis0.controller.config.vel_gain", g);
    }
    else if (strncmp(arg, "vi ", 3) == 0) {
      float g = (float)atof(arg + 3);
      Serial.printf("[CMD] Setting vel_integrator_gain to %.6f...\n", g);
      odrive.writeFloatProperty("axis0.controller.config.vel_integrator_gain", g);
    } else {
      Serial.println("[USAGE] gain pos <val> | gain vel <val> | gain vi <val>");
      return;
    }
    delay(50);
    printParametersDashboard();
  }
  else if (strncmp(line, "ilim ", 5) == 0) {
    const char* arg = line + 5;
    while (*arg == ' ') arg++;
    float limit = (float)atof(arg);
    Serial.printf("[CMD] Setting current limit to %.2f A...\n", limit);
    odrive.writeFloatProperty("axis0.config.motor.current_soft_max", limit);
    odrive.writeFloatProperty("axis0.motor.config.current_lim", limit);
    delay(50);
    printParametersDashboard();
  }
  // Flash & Diagnostics
  else if (strcmp(line, "save") == 0) {
    Serial.println("[CMD] Saving configuration to ODrive NVS Flash ('ss')...");
    odrive.sendRawCommand("ss");
    Serial.println("  -> Configuration saved to flash memory.");
  }
  else if (strcmp(line, "reboot") == 0) {
    Serial.println("[CMD] Requesting ODrive system reboot ('sr')...");
    odrive.sendRawCommand("sr");
    Serial.println("  -> ODrive rebooting. Wait 3-5 seconds then run 'diag' or 't'.");
  }
  else if (strcmp(line, "errors") == 0) {
    printErrorsDetailed();
  }
  else if (strcmp(line, "t") == 0) {
    printTelemetrySnapshot();
  }
  else if (strcmp(line, "m") == 0) {
    streamTelemetry = !streamTelemetry;
    Serial.printf("[CMD] Periodic Telemetry Stream %s\n", streamTelemetry ? "ENABLED (500ms)" : "DISABLED");
  }
  else if (strcmp(line, "diag") == 0) {
    runDiagnostics();
  }
  else if (strcmp(line, "swap") == 0) {
    uint8_t tmp = currentRxPin;
    currentRxPin = currentTxPin;
    currentTxPin = tmp;
    Serial.printf("[CMD] Swapping UART pins: RX = Pin %d, TX = Pin %d...\n", currentRxPin, currentTxPin);
    ODriveSerial.begin(currentBaud, SERIAL_8N1, currentRxPin, currentTxPin);
    delay(50);
    runDiagnostics();
  }
  else if (strncmp(line, "baud ", 5) == 0) {
    const char* arg = line + 5;
    while (*arg == ' ') arg++;
    uint32_t newBaud = (uint32_t)strtoul(arg, NULL, 10);
    if (newBaud > 0) {
      currentBaud = newBaud;
      Serial.printf("[CMD] Changing UART baud rate to %u...\n", currentBaud);
      ODriveSerial.begin(currentBaud, SERIAL_8N1, currentRxPin, currentTxPin);
      delay(50);
      runDiagnostics();
    }
  }
  // Direct ASCII Read / Write Pass-through
  else if (line[0] == 'r' && line[1] == ' ') {
    char response[64] = {0};
    if (odrive.sendRawCommand(line, response, sizeof(response), 100)) {
      Serial.printf("[ODrive Reply]: %s\n", response);
    } else {
      Serial.println("[ODrive Reply]: (No response / timeout)");
    }
  }
  else if (line[0] == 'w' && line[1] == ' ') {
    Serial.printf("[CMD-WRITE] %s\n", line);
    odrive.sendRawCommand(line);
  }
  else if (strncmp(line, "raw ", 4) == 0) {
    const char* rawCmd = line + 4;
    while (*rawCmd == ' ') rawCmd++;
    char response[64] = {0};
    if (odrive.sendRawCommand(rawCmd, response, sizeof(response), 200)) {
      Serial.printf("[ODrive Reply]: %s\n", response);
    } else {
      Serial.println("[ODrive Reply]: (No response / timeout)");
    }
  }
  else {
    Serial.printf("[UNKNOWN] Command '%s'. Type 'h' or '?' for available commands.\n", line);
  }
}

void setup() {
  Serial.begin(115200);

  unsigned long start = millis();
  while (!Serial && (millis() - start < 2000)) {
    delay(10);
  }

  Serial.println("\n\n=========================================================================");
  Serial.println("           ODrive S1 Advanced Engineering Serial Console                 ");
  Serial.println("=========================================================================");

  odrive.begin();

  printMenu();
  Serial.println("[Ready] Type 'params' for configuration, 'h' for menu, or 't' for status.");
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      inputBuffer[inputIdx] = '\0';
      if (inputIdx > 0) {
        handleCommand(inputBuffer);
        inputIdx = 0;
      }
    } else if (inputIdx < sizeof(inputBuffer) - 1) {
      inputBuffer[inputIdx++] = c;
    }
  }

  if (streamTelemetry) {
    unsigned long now = millis();
    if (now - lastStreamMs >= STREAM_INTERVAL_MS) {
      lastStreamMs = now;
      ODriveTelemetry telem;
      if (odrive.pollTelemetry(telem)) {
        Serial.printf("[STREAM] %.1fV | %.2fA | %.1fW | %.0f RPM | State: %s | Mode: %s (%s) | Err: 0x%X\n",
                      telem.vbus_voltage,
                      telem.ibus_current,
                      telem.power_watts,
                      telem.actual_rpm,
                      stateToString(telem.current_state),
                      controlModeToString(telem.control_mode),
                      inputModeToString(telem.input_mode),
                      telem.axis_error);
      } else {
        Serial.println("[STREAM] Communication timeout (no reply from ODrive)");
      }
    }
  }

  delay(5);
}
