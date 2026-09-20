#include "esp_now_rx.h"
#include "esp_now_packet.h"
#include <esp_now.h>
#include <WiFi.h>

static ControlEngine* s_controlEngine = nullptr;

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

  if (s_controlEngine != nullptr) {
    s_controlEngine->updateProcessVariable(pkt->dissolved_oxygen, pkt->water_temperature);
  }
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
  return true;
}
