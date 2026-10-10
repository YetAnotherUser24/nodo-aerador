#pragma once
#include <Arduino.h>
#include "control_engine.h"

struct LoggerSyncState {
  uint64_t rtc_timestamp_ms = 0; // epoch ms UTC, from the od-logger over ESP-NOW (schema t_dev_ms)
  uint32_t sequence = 0;          // Packet sequence counter
  float last_do_mg_l = 0.0f;
  float last_water_temp_c = 0.0f;
  float last_saturation_pct = 0.0f;
  float last_battery_v = 0.0f;
  uint32_t received_at_ms = 0;    // Local controller millis() when packet arrived
  uint32_t packet_count = 0;      // Total valid packets accepted from the OD-Logger
  bool sample_valid = false;      // false = el logger marcó la medición como fallida (status_flags)
  bool has_valid_packet = false;
};

/**
 * @brief Initializes ESP-NOW receiver on Controller node.
 * Automatically hooks incoming DO telemetry packets directly into the control engine.
 */
bool espNowRxInit(ControlEngine& engine);

/**
 * @brief Returns a thread-safe snapshot of the synchronized state and RTC timestamp from od-logger.
 */
LoggerSyncState espNowGetLoggerState();
