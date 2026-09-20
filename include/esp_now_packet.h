#pragma once
#include <stdint.h>

#define AQUACONTROL_PACKET_MAGIC 0x41515541 // ASCII "AQUA"

#pragma pack(push, 1)
struct OdTelemetryPacket {
  uint32_t magic;           // AQUACONTROL_PACKET_MAGIC
  uint32_t sequence;        // Incremental sequence counter
  uint32_t timestamp_sec;   // Seconds since 2000 (RTC)
  float dissolved_oxygen;   // mg/L (e.g. 7.850)
  float water_temperature;  // °C (e.g. 23.45)
  float oxygen_saturation;  // % Sat (e.g. 98.20)
  float battery_voltage;    // V (e.g. 4.18)
  uint32_t status_flags;    // Status bitfield
};
#pragma pack(pop)
