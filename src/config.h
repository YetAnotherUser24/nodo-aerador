#pragma once
#include <Arduino.h>
#include "board.h"

/* ----------------------------- WiFi ----------------------------- */
#define WIFI_SSID               "WifiDev"
#define WIFI_PASSWORD           "rpidev24"
#define WIFI_CONNECT_TIMEOUT_MS 15000

/* ----------------------- AquaControl Cloud ----------------------- */
#define AQUACONTROL_HOST        "tesisutec.vercel.app"
#define AQUACONTROL_MOTOR_PATH  "/api/telemetry/motor_bulk"
#define AQUACONTROL_PENDING_CMD "/api/commands/pending"
#define AQUACONTROL_ACK_PATH    "/api/commands" // + "/[id]/acknowledge"
#define AQUACONTROL_DEVICE_KEY  "ESP32_ODRIVE"

/* --------------------- Hardware UART (ODrive S1) ---------------- */
#ifndef ODRIVE_SERIAL_TX_PIN
#define ODRIVE_SERIAL_TX_PIN    13
#endif

#ifndef ODRIVE_SERIAL_RX_PIN
#define ODRIVE_SERIAL_RX_PIN    12
#endif

#ifndef ODRIVE_BAUD_RATE 
#define ODRIVE_BAUD_RATE        115200   // ODrive S1 default UART baud rate (compatible with ESP32-S3 and LilyGO HAT)
#endif

#ifndef ODRIVE_UART_DEBUG
#define ODRIVE_UART_DEBUG       0        // 1 = Log raw UART commands to Serial, 0 = Clean/Quiet
#endif

/* -------------------- ODrive Motion & Limits --------------------- */
#ifndef ODRIVE_CONTROL_MODE
#define ODRIVE_CONTROL_MODE     2        // 2 = VELOCITY_CONTROL
#endif

#ifndef ODRIVE_INPUT_MODE
#define ODRIVE_INPUT_MODE       2        // 2 = VEL_RAMP (Ramped Velocity Control)
#endif

#ifndef ODRIVE_VEL_RAMP_RATE
#define ODRIVE_VEL_RAMP_RATE    10.0f    // Maximum acceleration: 10 rev/s^2 (turns/s^2)
#endif

/* -------------------- Aeration Control Defaults ----------------- */
#define DEFAULT_TARGET_DO_MG_L  7.5f     // Target dissolved oxygen setpoint
#define DEFAULT_KP              100.0f
#define DEFAULT_KI              1.5f
#define DEFAULT_KD              10.0f

#define MIN_MOTOR_RPM           0.0f
#define MAX_MOTOR_RPM           600.0f
#define FAILSAFE_MOTOR_RPM      0.0f     // Baseline safety aeration if sensor drops (0 RPM default safe)
#define FAILSAFE_TIMEOUT_MS     35000UL  // Fall back to failsafe after 35s without DO packet

// Tensión mínima de bus para (re)armar el ODrive.
// Antes la condición de re-armado era `vbus_voltage > 12.0f`, es decir EXACTAMENTE la tensión
// nominal del banco (fuente de 12 VDC, AGENTS.md regla 12). Con el bus en reposo a ~12.0 V y
// cualquier caída por consumo, la comparación quedaba por debajo del umbral y el ODrive NO volvía
// a armarse tras un E-Stop o un desarme: el motor se quedaba a 0 RPM de forma permanente y sin
// ningún aviso. El umbral se baja a un valor holgadamente inferior al nominal.
#define ODRIVE_ARM_MIN_VBUS     8.0f

/* ------------------------ Timing Intervals ----------------------- */
#define CONTROL_LOOP_INTERVAL_MS 50UL         // 20 Hz deterministic loop on Core 1
#define TELEMETRY_PUSH_INTERVAL_MS 5000UL     // 5-second cloud push on Core 0
#define COMMAND_POLL_INTERVAL_MS   1500UL     // 1.5-second cloud command check in Auto mode (PID / Fuzzy)
#define COMMAND_POLL_INTERVAL_MANUAL_MS 1000UL // 1-second cloud command check in Manual mode

// New: telemetry sample interval (~3 Hz) to build batch of 9 samples per push
#define TELEMETRY_SAMPLE_INTERVAL_MS 200UL // 5 Hz sample interval to build batch of 25 samples 

/* -------------------- MicroSD Card Logger ------------------------ */
#ifndef SD_LOG_ENABLED
#if defined(BOARD_LILYGO_LORA32) && (!defined(ODRIVE_VIRTUAL) || !ODRIVE_VIRTUAL)
#define SD_LOG_ENABLED          1        // Enable high-speed SD logging on LilyGO hardware
#else
#define SD_LOG_ENABLED          1
#endif
#endif

#define SD_LOG_INTERVAL_MS      200UL    // 5 Hz SD sampling rate on Core 1 (V4 §2.2, solo en experimento)
#define SD_LOG_QUEUE_SIZE       64       // RAM queue holding up to 6.4s of samples during flash erase
#define SD_LOG_FLUSH_BATCH_SIZE 10       // Flush after 10 samples (every 1 second) or idle

/* ----------------- Virtual ODrive Simulator Settings ------------- */
#ifndef ODRIVE_VIRTUAL
#define ODRIVE_VIRTUAL          0        // 1 = Simulation mode (no physical ODrive / no ESP-NOW), 0 = Real hardware
#endif
#define VIRTUAL_VBUS_NOMINAL    24.0f    // Simulated nominal DC bus voltage (V)
#define VIRTUAL_RAMP_RPM_PER_S  600.0f   // Simulated acceleration ramp rate: 10 rev/s^2 (600 RPM/s)
#define VIRTUAL_IBUS_IDLE       0.15f    // Quiescent driver current (A)
#define VIRTUAL_IBUS_MAX_LOAD   9.5f     // Max continuous motor current at 3500 RPM under water load (A)

/* ------------------------ Over-The-Air (OTA) --------------------- */
#ifndef OTA_ENABLED
#define OTA_ENABLED             1        // 1 = Enable ArduinoOTA over WiFi, 0 = Disable
#endif
#define OTA_HOSTNAME            "aquacontrol-odrive" // mDNS name (aquacontrol-odrive.local)
#define OTA_PORT                3232     // Default ESP32 OTA port
#define OTA_PASSWORD            ""       // Optional authentication password (empty = no password)

/* ------------------------ WiFi Telnet Stream --------------------- */
#ifndef TELNET_LOG_ENABLED
#define TELNET_LOG_ENABLED      0       // 1 = Enable remote Telnet terminal over WiFi, 0 = Disable
#endif
#define TELNET_LOG_PORT         23       // Standard Telnet port


