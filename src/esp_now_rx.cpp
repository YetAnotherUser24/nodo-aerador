#include "esp_now_rx.h"
#include "esp_now_packet.h"
#include <esp_now.h>
#include <WiFi.h>

static ControlEngine* s_controlEngine = nullptr;
static LoggerSyncState s_loggerSyncState;
static portMUX_TYPE s_syncMux = portMUX_INITIALIZER_UNLOCKED;

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
static void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#else
static void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  if (len != sizeof(OdTelemetryPacket)) {
    return;
  }

  const OdTelemetryPacket* pkt = reinterpret_cast<const OdTelemetryPacket*>(data);
  if (pkt->magic != AQUACONTROL_PACKET_MAGIC) {
    return;
  }

  // El logger marca con un bit de `status_flags` una medición fallida (sensor Modbus sin respuesta).
  // Esa muestra NO es un 0 mg/L real: aceptarla haría que el PID persiguiera un error enorme.
  const bool sampleValid =
      (pkt->status_flags & (1UL << AQUACONTROL_STATUS_BIT_OD_SENSOR)) == 0;

  portENTER_CRITICAL(&s_syncMux);
  s_loggerSyncState.rtc_timestamp_ms = pkt->t_dev_ms;
  s_loggerSyncState.sequence = pkt->sequence;
  s_loggerSyncState.last_do_mg_l = pkt->dissolved_oxygen;
  s_loggerSyncState.last_water_temp_c = pkt->water_temperature;
  s_loggerSyncState.last_saturation_pct = pkt->oxygen_saturation;
  s_loggerSyncState.last_battery_v = pkt->battery_voltage;
  s_loggerSyncState.received_at_ms = millis();
  s_loggerSyncState.packet_count++;
  s_loggerSyncState.sample_valid = sampleValid;
  s_loggerSyncState.has_valid_packet = true;
  portEXIT_CRITICAL(&s_syncMux);

  // Al no refrescar la variable de proceso, el watchdog de 35 s detiene el motor en lugar de
  // girar a fondo contra un DO inválido. Es el comportamiento seguro ante falla de sensor.
  if (s_controlEngine != nullptr && sampleValid) {
    s_controlEngine->updateProcessVariable(pkt->dissolved_oxygen, pkt->water_temperature);
  }
}

LoggerSyncState espNowGetLoggerState() {
  portENTER_CRITICAL(&s_syncMux);
  LoggerSyncState copy = s_loggerSyncState;
  portEXIT_CRITICAL(&s_syncMux);
  return copy;
}

bool espNowRxInit(ControlEngine& engine) {
  s_controlEngine = &engine;

  if (WiFi.getMode() == WIFI_OFF) {
    WiFi.mode(WIFI_STA);
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW RX] Error initializing ESP-NOW");
    return false;
  }

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
  esp_now_register_recv_cb(onDataRecv);
#else
  esp_now_register_recv_cb(onDataRecv);
#endif

  Serial.println("[ESP-NOW RX] Receiver listening for DO packets.");
  // El receptor NO registra peers: recibe en el canal actual del STA. Si el canal reportado aquí
  // no coincide con el del od-logger, los paquetes nunca llegarán (ambos deben seguir al mismo AP).
  Serial.printf("[ESP-NOW RX] Ready. WiFi ch=%u | packet=%u bytes (peer-less receive, follows the AP channel).\n",
                (unsigned)WiFi.channel(), (unsigned)sizeof(OdTelemetryPacket));
  return true;
}
