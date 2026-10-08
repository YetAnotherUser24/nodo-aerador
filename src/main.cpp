/**
 * @file main.cpp
 * @brief AquaControl: ESP32 Closed-Loop ODrive S1 Aeration Controller
 *
 * Architecture:
 *   Core 1: Deterministic 20 Hz Control Loop (PID / Fuzzy / Watchdog / Manual)
 *           Sole hardware owner of ODrive S1 UART (Zero Cross-Core Contention)
 *   Core 0: FreeRTOS Cloud Worker (WiFi + HTTPS Supabase telemetry + Command Polling + NVS Flash)
 *   P2P: Direct ESP-NOW link receiving DO & Temperature from OD-Logger
 */

#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "odrive_uart.h"
#include "control_engine.h"
#include "esp_now_rx.h"
#include "cloud_worker.h"
#include "odrive_sd_logger.h"

#if defined(ODRIVE_VIRTUAL) && ODRIVE_VIRTUAL
#include "odrive_virtual.h"
static ODriveVirtual odrive(VIRTUAL_VBUS_NOMINAL, VIRTUAL_RAMP_RPM_PER_S);
#else
// Instantiate ODrive UART driver on Serial2 (or Serial1 on S3)
#if CONFIG_IDF_TARGET_ESP32S3
static HardwareSerial ODriveSerial(1);
#else
static HardwareSerial ODriveSerial(2);
#endif

static ODriveUART odrive(ODriveSerial, ODRIVE_SERIAL_RX_PIN, ODRIVE_SERIAL_TX_PIN, ODRIVE_BAUD_RATE);
#endif

static ControlEngine controlEngine;

static unsigned long lastLoopMs = 0;
static unsigned long lastLogMs = 0;
static unsigned long lastSdLogMs = 0;
static uint32_t loopTickCounter = 0;
static ODriveTelemetry cachedODriveTelem;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=======================================================");
#if defined(ODRIVE_VIRTUAL) && ODRIVE_VIRTUAL
  Serial.println("  AquaControl: ESP32 [VIRTUAL ODRIVE] Controller Ready  ");
#else
  Serial.println("  AquaControl: ESP32 ODrive S1 Controller Starting...  ");
#endif
  Serial.printf("  Board: %s\n", BOARD_NAME);
  Serial.println("=======================================================");

  // 1. Initialize ODrive Driver (Virtual Simulator or Hardware UART)
#if defined(ODRIVE_VIRTUAL) && ODRIVE_VIRTUAL
  Serial.println("[Init] Initializing ODrive Virtual Driver (Simulation Mode)...");
#else
  Serial.printf("[Init] Connecting to ODrive S1 via UART (RX: GPIO %d, TX: GPIO %d @ %lu baud)...\n",
                ODRIVE_SERIAL_RX_PIN, ODRIVE_SERIAL_TX_PIN, (unsigned long)ODRIVE_BAUD_RATE);
#endif
  odrive.begin();

  // 2. Initialize ESP-NOW Receiver (P2P direct radio from Logger)
#if !defined(ODRIVE_VIRTUAL) || !ODRIVE_VIRTUAL
  Serial.println("[Init] Initializing ESP-NOW radio...");
  if (!espNowRxInit(controlEngine)) {
    Serial.println("[Init] WARNING: ESP-NOW init failed! Controller will use failsafe baseline.");
  }
#else
  Serial.println("[Init] Virtual Mode: ESP-NOW radio disabled (focusing on Web Dashboard <-> ODrive link).");
#endif

  // 3. Initialize MicroSD Card Data Logger
  odriveSdLoggerInit();

  // 4. Start Core 0 Cloud Worker (Passes only ControlEngine, preserving UART single ownership)
  Serial.println("[Init] Launching Cloud Worker on Core 0...");
  startCloudWorker(controlEngine);

  // 5. Initial attempt to engage closed-loop control on ODrive S1
  odrive.clearErrors();
  delay(100);
  odrive.setControlMode(ODRIVE_CONTROL_MODE, ODRIVE_INPUT_MODE);
  delay(50);
  odrive.writeFloatProperty("axis0.controller.config.vel_ramp_rate", ODRIVE_VEL_RAMP_RATE);
  delay(50);
  odrive.enterClosedLoop();
  delay(100);
  if (odrive.pollTelemetry(cachedODriveTelem)) {
    Serial.printf("[Init] ODrive connected: %.1fV | State: %u (%s) | Mode: VEL_RAMP (%.1f rev/s^2)\n",
                  cachedODriveTelem.vbus_voltage,
                  cachedODriveTelem.current_state,
                  cachedODriveTelem.is_armed ? "ARMED" : "DISARMED",
                  ODRIVE_VEL_RAMP_RATE);
  } else {
    Serial.println("[Init] WARNING: ODrive not responding on UART! Check wiring & power.");
  }

  Serial.println("[Init] System ready. Deterministic control loop active on Core 1.\n");
}

void loop() {
  unsigned long now = millis();

  // 20 Hz deterministic control loop on Core 1
  if (now - lastLoopMs >= CONTROL_LOOP_INTERVAL_MS) {
    lastLoopMs = now;

    // A. Query live telemetry via Tiered Polling (strictly <= 2.5 ms serial time per tick)
    odrive.updateTieredTelemetry(cachedODriveTelem, loopTickCounter++);

    // B. Check Emergency Stop or compute target velocity from control algorithm
    float targetRpm = 0.0f;
    if (controlEngine.isEmergencyStopActive()) {
      targetRpm = 0.0f;
      odrive.setVelocityRpm(0.0f);
      odrive.enterIdle();
    } else {
      // If disarmed (e.g. recovering from e-stop or power-on) but DC bus is alive, re-arm
      if (!cachedODriveTelem.is_armed && cachedODriveTelem.vbus_voltage > 12.0f) {
        odrive.enterClosedLoop();
      }
      targetRpm = controlEngine.computeOutputRpm(now);
      odrive.setVelocityRpm(targetRpm);
    }

    const LoggerSyncState& sync = espNowGetLoggerState();

    // C. Build shared telemetry snapshot for Core 0 cloud worker
    SharedTelemetry sTelem;
    if (sync.has_valid_packet) {
      sTelem.rtc_timestamp_ms = sync.rtc_timestamp_ms + (now - sync.received_at_ms);
    } else {
      sTelem.rtc_timestamp_ms = 0; // Will be ignored by backend if 0
    }
    sTelem.target_rpm = targetRpm;
    sTelem.actual_rpm = cachedODriveTelem.actual_rpm;
    sTelem.vbus_voltage = cachedODriveTelem.vbus_voltage;
    sTelem.ibus_current = cachedODriveTelem.ibus_current;
    sTelem.power_watts = cachedODriveTelem.power_watts;
    sTelem.torque_estimate = cachedODriveTelem.torque_estimate;
    sTelem.iq_measured = cachedODriveTelem.iq_measured;
    sTelem.measured_do = controlEngine.getLastMeasuredDo();
    sTelem.water_temp = controlEngine.getLastWaterTemp();
    sTelem.is_on = (targetRpm > 10.0f);
    sTelem.control_mode = (uint8_t)controlEngine.getConfig().mode;
    sTelem.failsafe_active = controlEngine.isFailsafeActive();
    sTelem.status_code = cachedODriveTelem.axis_error;

    updateSharedTelemetry(sTelem);

    // D. MicroSD logging a 5 Hz SOLO durante un experimento activo (V4 §2.1/§2.2)
    if (controlEngine.isExperimentLogging() && now - lastSdLogMs >= SD_LOG_INTERVAL_MS) {
      lastSdLogMs = now;

      OdriveLogSample sample;
      sample.t_ms = now;
      sample.rtc_timestamp_ms = sTelem.rtc_timestamp_ms;
      sample.target_rpm = targetRpm;
      sample.actual_rpm = cachedODriveTelem.actual_rpm;
      sample.torque_nm = cachedODriveTelem.torque_estimate;
      sample.iq_a = cachedODriveTelem.iq_measured;
      sample.ibus_a = cachedODriveTelem.ibus_current;
      sample.vbus_v = cachedODriveTelem.vbus_voltage;
      sample.fet_temp_c = cachedODriveTelem.fet_temperature;
      sample.current_do_mg_l = controlEngine.getLastMeasuredDo();
      sample.control_mode = (uint8_t)controlEngine.getConfig().mode;

      odriveSdLoggerEnqueue(sample);
    }

    // E. Periodic Serial Monitor status output (every 1 second)
    if (now - lastLogMs >= 1000) {
      lastLogMs = now;
      ControlConfig cfg = controlEngine.getConfig();
      const char* modeStr = (cfg.mode == MODE_PID) ? "PID" :
                            ((cfg.mode == MODE_FUZZY) ? "FUZZY" : "MANUAL");

      remoteLogPrintf("[CTRL Core 1] Mode: %s | DO: %.2f mg/L | Target: %.0f RPM | Act: %.0f RPM | %.1fV %.2fA (%.1fW) | Iq: %.3fA | ch=%u espnow_rx=%u%s%s%s%s%s\n",
                    modeStr,
                    controlEngine.getLastMeasuredDo(),
                    targetRpm,
                    cachedODriveTelem.actual_rpm,
                    cachedODriveTelem.vbus_voltage,
                    cachedODriveTelem.ibus_current,
                    cachedODriveTelem.power_watts,
                    cachedODriveTelem.torque_estimate,
                    (unsigned)WiFi.channel(),
                    (unsigned)sync.packet_count,
                    controlEngine.isFailsafeActive() ? " [FAILSAFE]" : "",
                    controlEngine.isEmergencyStopActive() ? " [EMERGENCY STOP]" : "",
                    (!cachedODriveTelem.is_armed) ? " [DISARMED]" : "",
                    (cachedODriveTelem.axis_error != 0) ? " [ERROR]" : "",
                    (sync.has_valid_packet && !sync.sample_valid) ? " [OD INVALID]" : "");
    }
  }

  // Yield to allow FreeRTOS IDLE task watchdog reset on Core 1
  vTaskDelay(pdMS_TO_TICKS(5));
}
