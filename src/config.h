#pragma once
#include <Arduino.h>
#include "board.h"

/* ----------------------------- WiFi ----------------------------- */
#define WIFI_SSID               "WifiDev"
#define WIFI_PASSWORD           "rpidev24"
#define WIFI_CONNECT_TIMEOUT_MS 15000

/* ----------------------- AquaControl Cloud ----------------------- */
#define AQUACONTROL_HOST        "tesisutec.vercel.app"
#define AQUACONTROL_MOTOR_PATH  "/api/telemetry/motor"
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
#if defined(BOARD_LILYGO_LORA32)
#define ODRIVE_BAUD_RATE        460800   // High-speed low-latency UART for LilyGO HAT
#else
#define ODRIVE_BAUD_RATE        115200
#endif
#endif

#ifndef ODRIVE_UART_DEBUG
#define ODRIVE_UART_DEBUG       0        // 1 = Log raw UART commands to Serial, 0 = Clean/Quiet
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

/* ------------------------ Timing Intervals ----------------------- */
#define CONTROL_LOOP_INTERVAL_MS 50UL         // 20 Hz deterministic loop on Core 1
#define TELEMETRY_PUSH_INTERVAL_MS 3000UL     // 3-second cloud push on Core 0
#define COMMAND_POLL_INTERVAL_MS   2000UL     // 2-second cloud command check in Auto mode (PID / Fuzzy)
#define COMMAND_POLL_INTERVAL_MANUAL_MS 500UL // 500ms fast cloud command check in Manual mode

// New: telemetry sample interval (~3 Hz) to build batch of 9 samples per push
#define TELEMETRY_SAMPLE_INTERVAL_MS 350UL 

/* -------------------- MicroSD Card Logger ------------------------ */
#ifndef SD_LOG_ENABLED
#if defined(BOARD_LILYGO_LORA32) && (!defined(ODRIVE_VIRTUAL) || !ODRIVE_VIRTUAL)
#define SD_LOG_ENABLED          1        // Enable high-speed SD logging on LilyGO hardware
#else
#define SD_LOG_ENABLED          0
#endif
#endif

#define SD_LOG_INTERVAL_MS      100UL    // 10 Hz SD sampling rate on Core 1
#define SD_LOG_QUEUE_SIZE       64       // RAM queue holding up to 6.4s of samples during flash erase
#define SD_LOG_FLUSH_BATCH_SIZE 10       // Flush after 10 samples (every 1 second) or idle

/* ----------------- Virtual ODrive Simulator Settings ------------- */
#ifndef ODRIVE_VIRTUAL
#define ODRIVE_VIRTUAL          0        // 1 = Simulation mode (no physical ODrive / no ESP-NOW), 0 = Real hardware
#endif
#define VIRTUAL_VBUS_NOMINAL    24.0f    // Simulated nominal DC bus voltage (V)
#define VIRTUAL_RAMP_RPM_PER_S  1500.0f  // Simulated acceleration ramp rate (RPM/s)
#define VIRTUAL_IBUS_IDLE       0.15f    // Quiescent driver current (A)
#define VIRTUAL_IBUS_MAX_LOAD   9.5f     // Max continuous motor current at 3500 RPM under water load (A)
