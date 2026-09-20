#pragma once
#include <Arduino.h>
#include "control_engine.h"

// Thread-safe shared snapshot populated by Core 1 and read by Core 0
struct SharedTelemetry {
  float target_rpm = 0.0f;
  float actual_rpm = 0.0f;
  float vbus_voltage = 0.0f;
  float ibus_current = 0.0f;
  float power_watts = 0.0f;
  float measured_do = 0.0f;
  float water_temp = 0.0f;
  bool is_on = false;
  uint8_t control_mode = 0; // 0=PID, 1=Fuzzy, 2=Manual
  bool failsafe_active = false;
  uint32_t status_code = 0;
};

void updateSharedTelemetry(const SharedTelemetry& telem);

/**
 * @brief Spawns the background cloud worker task pinned to Core 0.
 * Core 0 interacts strictly with ControlEngine (never directly with hardware UART).
 */
void startCloudWorker(ControlEngine& engine);
