#pragma once
#include <Arduino.h>
#include "control_engine.h"

struct LoggerSyncState {
  uint64_t rtc_timestamp_ms = 0; // DS3231 RTC milliseconds from od-logger
  uint32_t sequence = 0;          // Packet sequence counter
  float last_do_mg_l = 0.0f;
  float last_water_temp_c = 0.0f;
  float last_saturation_pct = 0.0f;
  float last_battery_v = 0.0f;
  uint32_t received_at_ms = 0;    // Local controller millis() when packet arrived
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
