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
  // Unix epoch milliseconds, UTC -- schema v2 `t_dev_ms`, and the same base the NTP nodes use.
  //
  // This field was previously documented here as "milliseconds since 2000", which the sender
  // never wrote and the receiver never converted. The mismatch was invisible only because both
  // ends agreed by accident: a comment describing the wrong units is an invitation for someone
  // to "fix" the reader into converting a value that is already correct, which is the exact
  // guess `docs/TIME-SYNC.md` §3 retires by name. Hence the field is no longer called
  // `rtc_timestamp_ms`.
  uint64_t t_dev_ms;
  float dissolved_oxygen;   // mg/L (e.g. 7.850)
  float water_temperature;  // °C (e.g. 23.45)
  float oxygen_saturation;  // % Sat (e.g. 98.20)
  float battery_voltage;    // V (e.g. 4.18)
  uint32_t status_flags;    // Status bitfield
};
#pragma pack(pop)
