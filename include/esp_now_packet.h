#pragma once
#include <stdint.h>

#define AQUACONTROL_PACKET_MAGIC 0x41515541 // ASCII "AQUA"

/*
 * Espejo de los bits `STATUS_*` de od-logger/src/n_logger_config.h. El receptor debe respetarlos:
 * un bit activo significa que esa medición NO es válida y no debe entrar al lazo de control.
 * (El logger transmite DO=0.000 con este bit activo cuando la lectura Modbus falla.)
 */
#define AQUACONTROL_STATUS_BIT_OD_SENSOR 22

#pragma pack(push, 1)
struct OdTelemetryPacket {
  uint32_t magic;           // AQUACONTROL_PACKET_MAGIC
  uint32_t sequence;        // Incremental sequence counter
  uint64_t rtc_timestamp_ms; // Milliseconds since 2000 (RTC + millis() offset)
  float dissolved_oxygen;   // mg/L (e.g. 7.850)
  float water_temperature;  // °C (e.g. 23.45)
  float oxygen_saturation;  // % Sat (e.g. 98.20)
  float battery_voltage;    // V (e.g. 4.18)
  uint32_t status_flags;    // Status bitfield
};
#pragma pack(pop)
