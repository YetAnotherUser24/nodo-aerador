#include "cloud_worker.h"
#include "config.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <lwip/sockets.h>

static SharedTelemetry s_sharedTelem;
static portMUX_TYPE s_telemMux = portMUX_INITIALIZER_UNLOCKED;
static ControlEngine* s_engine = nullptr;
static Preferences s_prefs;

// Batch telemetry buffer (9 samples per push)
static const uint8_t TELEMETRY_BATCH_SIZE = 9;
static SharedTelemetry s_telemetryBatch[TELEMETRY_BATCH_SIZE];
static uint8_t s_batchIndex = 0;
static uint8_t s_batchCount = 0; // number of valid samples

void updateSharedTelemetry(const SharedTelemetry& telem) {
  portENTER_CRITICAL(&s_telemMux);
  s_sharedTelem = telem;
  portEXIT_CRITICAL(&s_telemMux);
}

static SharedTelemetry getSharedTelemetry() {
  SharedTelemetry copy;
  portENTER_CRITICAL(&s_telemMux);
  copy = s_sharedTelem;
  portEXIT_CRITICAL(&s_telemMux);
  return copy;
}

static void loadPersistedConfig(ControlEngine& engine) {
  s_prefs.begin("aquactrl", false);
  ControlConfig cfg = engine.getConfig();
  if (s_prefs.isKey("mode")) cfg.mode = (ControlMode)s_prefs.getUChar("mode", (uint8_t)cfg.mode);
  if (s_prefs.isKey("target_do")) cfg.target_do_mg_l = s_prefs.getFloat("target_do", cfg.target_do_mg_l);
  if (s_prefs.isKey("kp")) cfg.kp = s_prefs.getFloat("kp", cfg.kp);
  if (s_prefs.isKey("ki")) cfg.ki = s_prefs.getFloat("ki", cfg.ki);
  if (s_prefs.isKey("kd")) cfg.kd = s_prefs.getFloat("kd", cfg.kd);
  if (s_prefs.isKey("failsafe_rpm")) cfg.failsafe_rpm = s_prefs.getFloat("failsafe_rpm", cfg.failsafe_rpm);
  s_prefs.end();

  engine.configure(cfg);
  Serial.printf("[NVS] Loaded config: mode=%d | target_do=%.2f | kp=%.1f ki=%.2f kd=%.1f\n",
                cfg.mode, cfg.target_do_mg_l, cfg.kp, cfg.ki, cfg.kd);
}

static void savePersistedConfig(const ControlConfig& cfg) {
  s_prefs.begin("aquactrl", false);
  s_prefs.putUChar("mode", (uint8_t)cfg.mode);
  s_prefs.putFloat("target_do", cfg.target_do_mg_l);
  s_prefs.putFloat("kp", cfg.kp);
  s_prefs.putFloat("ki", cfg.ki);
  s_prefs.putFloat("kd", cfg.kd);
  s_prefs.putFloat("failsafe_rpm", cfg.failsafe_rpm);
  s_prefs.end();
  Serial.println("[NVS] Persisted updated control parameters to Flash.");
}

static void closeClientSocket(WiFiClientSecure& client) {
  int fd = client.fd();
  if (fd >= 0) {
    struct linger sl = { 1, 0 }; // sends RST on close, prevents 60s TIME_WAIT descriptor exhaustion
    setsockopt(fd, SOL_SOCKET, SO_LINGER, &sl, sizeof(sl));
  }
  client.stop();
}

// Single telemetry send (kept for compatibility, not used in batch mode)
static void sendTelemetry(WiFiClientSecure& client, const SharedTelemetry& telem) {
  HTTPClient http;
  String url = String("https://") + AQUACONTROL_HOST + AQUACONTROL_MOTOR_PATH;
  http.setReuse(true);
  if (!http.begin(client, url)) {
    Serial.println("[Cloud] Telemetry begin failed");
    return;
  }
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-Device-Key", AQUACONTROL_DEVICE_KEY);
  http.setTimeout(4000);

  float speedPercent = (telem.actual_rpm / MAX_MOTOR_RPM) * 100.0f;
  speedPercent = constrain(speedPercent, 0.0f, 100.0f);

  StaticJsonDocument<300> doc;
  doc["is_on"] = telem.is_on;
  doc["speed_percent"] = round(speedPercent * 10.0f) / 10.0f;
  doc["pwm_us"] = (int)telem.actual_rpm;
  doc["voltage_v"] = round(telem.vbus_voltage * 100.0f) / 100.0f;
  doc["current_a"] = round(telem.ibus_current * 100.0f) / 100.0f;
  doc["power_w"] = round(telem.power_watts * 10.0f) / 10.0f;
  doc["status_code"] = telem.status_code;

  String body;
  serializeJson(doc, body);
  int code = http.POST(body);
  if (code >= 200 && code < 300) {
    Serial.printf("[Cloud] Telemetry sent: %.0f RPM | %.1fV %.2fA (HTTP %d)\n",
                  telem.actual_rpm, telem.vbus_voltage, telem.ibus_current, code);
  } else {
    Serial.printf("[Cloud] Telemetry POST returned %d: %s\n", code, http.errorToString(code).c_str());
  }
  http.end();
  closeClientSocket(client);
}

// Batch telemetry send – sends an array of telemetry samples
static void sendTelemetryBatch(WiFiClientSecure& client, const SharedTelemetry* batch, uint8_t count) {
  if (count == 0) return; // nothing to send
  HTTPClient http;
  String url = String("https://") + AQUACONTROL_HOST + AQUACONTROL_MOTOR_PATH;
  http.setReuse(true);
  if (!http.begin(client, url)) {
    Serial.println("[Cloud] Telemetry batch begin failed");
    return;
  }
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-Device-Key", AQUACONTROL_DEVICE_KEY);
  http.setTimeout(4000);

  // Build JSON array of samples
  StaticJsonDocument<3000> doc; // enough for 9 samples
  JsonArray samples = doc.createNestedArray("samples");
  for (uint8_t i = 0; i < count; ++i) {
    JsonObject obj = samples.createNestedObject();
    float speedPercent = (batch[i].actual_rpm / MAX_MOTOR_RPM) * 100.0f;
    speedPercent = constrain(speedPercent, 0.0f, 100.0f);
    obj["is_on"] = batch[i].is_on;
    obj["speed_percent"] = round(speedPercent * 10.0f) / 10.0f;
    obj["pwm_us"] = (int)batch[i].actual_rpm;
    obj["voltage_v"] = round(batch[i].vbus_voltage * 100.0f) / 100.0f;
    obj["current_a"] = round(batch[i].ibus_current * 100.0f) / 100.0f;
    obj["power_w"] = round(batch[i].power_watts * 10.0f) / 10.0f;
    obj["status_code"] = batch[i].status_code;
  }

  String body;
  serializeJson(doc, body);
  int code = http.POST(body);
  if (code >= 200 && code < 300) {
    Serial.printf("[Cloud] Batch telemetry ( %d samples ) sent (HTTP %d)\n", count, code);
  } else {
    Serial.printf("[Cloud] Batch telemetry POST error %d: %s\n", code, http.errorToString(code).c_str());
  }
  http.end();
  closeClientSocket(client);
}

static void acknowledgeCommand(WiFiClientSecure& client, const String& cmdId, bool success, float speedPercent) {
  HTTPClient http;
  String url = String("https://") + AQUACONTROL_HOST + AQUACONTROL_ACK_PATH + "/" + cmdId + "/acknowledge";

  if (!http.begin(client, url)) {
    Serial.println("[Cloud] Acknowledge begin failed");
    return;
  }

  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-Device-Key", AQUACONTROL_DEVICE_KEY);
  http.setTimeout(4000);

  StaticJsonDocument<150> doc;
  doc["success"] = success;
  doc["actual_speed_percent"] = speedPercent;

  String body;
  serializeJson(doc, body);
  int code = http.POST(body);
  Serial.printf("[Cloud] Acknowledged cmd %s: HTTP %d\n", cmdId.c_str(), code);

  http.end();
  closeClientSocket(client);
}

static bool pollPendingCommands(WiFiClientSecure& client) {
  HTTPClient http;
  String url = String("https://") + AQUACONTROL_HOST + AQUACONTROL_PENDING_CMD + "?device_key=" + AQUACONTROL_DEVICE_KEY;

  if (!http.begin(client, url)) {
    Serial.println("[Cloud] Poll begin failed");
    return false;
  }

  http.addHeader("X-Device-Key", AQUACONTROL_DEVICE_KEY);
  http.setTimeout(4000);

  int code = http.GET();
  String payload;
  if (code == 200) {
    payload = http.getString();
  } else if (code < 0) {
    Serial.printf("[Cloud] Poll connection error: %s (%d)\n", http.errorToString(code).c_str(), code);
  } else if (code != 200) {
    Serial.printf("[Cloud] Poll returned HTTP %d\n", code);
  }

  // End HTTP poll session before processing actions or sending acknowledge POST
  http.end();
  closeClientSocket(client);

  if (code == 200 && payload.length() > 0) {
    String cmdId;
    String cmdType;
    float speedPct = 0.0f;
    bool hasCmd = false;
    bool isEmergency = false;
    bool isStop = false;
    bool isThrottle = false;
    bool isConfig = false;
    ControlConfig cfg = s_engine->getConfig();

    {
      StaticJsonDocument<512> doc;
      DeserializationError err = deserializeJson(doc, payload);
      if (!err && (doc["has_command"] == true || doc["has_command"].as<bool>())) {
        JsonObject cmd = doc["command"];
        cmdId = cmd["id"].as<String>();
        cmdType = cmd["command_type"].as<String>();
        speedPct = cmd["speed_percent"].as<float>();
        hasCmd = true;

        if (cmdType == "emergency_stop") {
          isEmergency = true;
        } else if (cmdType == "stop") {
          isStop = true;
        } else if (cmdType == "start" || cmdType == "set_speed") {
          isThrottle = true;
          if (cmdType == "start" && speedPct <= 0.0f) {
            speedPct = (cfg.manual_throttle_pct > 0.0f) ? cfg.manual_throttle_pct : 20.0f;
          }
        } else if (cmdType == "set_mode" || cmdType == "set_config") {
          isConfig = true;
          JsonObject p = cmd["payload"];
          if (!p.isNull()) {
            if (p.containsKey("mode")) {
              String m = p["mode"].as<String>();
              if (m == "pid") cfg.mode = MODE_PID;
              else if (m == "fuzzy") cfg.mode = MODE_FUZZY;
              else if (m == "manual") cfg.mode = MODE_MANUAL;
            }
            if (p.containsKey("target_do")) cfg.target_do_mg_l = p["target_do"].as<float>();
            if (p.containsKey("kp")) cfg.kp = p["kp"].as<float>();
            if (p.containsKey("ki")) cfg.ki = p["ki"].as<float>();
            if (p.containsKey("kd")) cfg.kd = p["kd"].as<float>();
            if (p.containsKey("manual_throttle_pct")) cfg.manual_throttle_pct = p["manual_throttle_pct"].as<float>();
          }
        }
      } else if (err) {
        Serial.printf("[Cloud] Command JSON parse error: %s\n", err.c_str());
      }
    } // doc is destroyed here, reclaiming 512B stack before acknowledgeCommand starts TLS

    if (hasCmd) {
      Serial.printf("[Cloud] Executing command '%s' (speed: %.1f%%)\n", cmdType.c_str(), speedPct);

      if (isEmergency) {
        s_engine->triggerEmergencyStop();
        acknowledgeCommand(client, cmdId, true, 0.0f);
      } else if (isStop) {
        s_engine->setManualThrottle(0.0f);
        acknowledgeCommand(client, cmdId, true, 0.0f);
      } else if (isThrottle) {
        s_engine->setManualThrottle(speedPct);
        acknowledgeCommand(client, cmdId, true, speedPct);
      } else if (isConfig) {
        s_engine->configure(cfg);
        savePersistedConfig(cfg);
        acknowledgeCommand(client, cmdId, true, speedPct);
      } else if (cmdType == "reboot") {
        acknowledgeCommand(client, cmdId, true, 0.0f);
        delay(500);
        ESP.restart();
      }
      return true;
    }
  }
  return false;
}

static void cloudWorkerTask(void* pvParameters) {
  Serial.println("[Cloud Worker] Task started on Core 0");

  WiFiClientSecure client;
  client.setInsecure(); // ponytail: skip cert pinning for serverless Vercel endpoints
  client.setTimeout(4);
  client.setHandshakeTimeout(5);

  unsigned long lastTelemPush = 0;
  unsigned long lastTelemSample = 0;
  unsigned long lastCmdPoll = 0;
  uint8_t fastPollBurst = 0;

  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.printf("[Cloud Worker] Connecting to WiFi '%s'...\n", WIFI_SSID);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      unsigned long t0 = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
        vTaskDelay(pdMS_TO_TICKS(500));
      }
      if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[Cloud Worker] WiFi connected. IP: %s (RSSI: %d dBm)\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
      } else {
        Serial.println("[Cloud Worker] WiFi connection timed out, will retry...");
      }
    }

    if (WiFi.status() == WL_CONNECTED) {
      unsigned long now = millis();

      if (now - lastTelemSample >= TELEMETRY_SAMPLE_INTERVAL_MS) {
        lastTelemSample = now;
        SharedTelemetry telem = getSharedTelemetry();
        s_telemetryBatch[s_batchIndex] = telem;
        s_batchIndex = (s_batchIndex + 1) % TELEMETRY_BATCH_SIZE;
        if (s_batchCount < TELEMETRY_BATCH_SIZE) s_batchCount++;
      }

      // Priority 1: Check cloud commands (dynamic interval: fast in Manual mode)
      unsigned long cmdPollInterval = (s_engine && s_engine->getConfig().mode == MODE_MANUAL)
                                        ? COMMAND_POLL_INTERVAL_MANUAL_MS
                                        : COMMAND_POLL_INTERVAL_MS;

      if (now - lastCmdPoll >= cmdPollInterval) {
        lastCmdPoll = now;
        bool hadCmd = pollPendingCommands(client);
        // Drain queued backlog immediately (capped at 3 consecutive fast polls to avoid server hammering)
        if (hadCmd && ++fastPollBurst < 3) {
          lastCmdPoll = 0;
        } else {
          fastPollBurst = 0;
        }
      }

      // Priority 2: Periodic batch telemetry push
      if (now - lastTelemPush >= TELEMETRY_PUSH_INTERVAL_MS) {
        lastTelemPush = now;
        // Send batch if we have at least one sample (send whatever is collected)
        if (s_batchCount > 0) {
          sendTelemetryBatch(client, s_telemetryBatch, s_batchCount);
          // Reset batch after sending
          s_batchCount = 0;
          s_batchIndex = 0;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void startCloudWorker(ControlEngine& engine) {
  s_engine = &engine;

  loadPersistedConfig(engine);

  xTaskCreatePinnedToCore(
    cloudWorkerTask,
    "CloudWorkerTask",
    16384,
    NULL,
    1,
    NULL,
    0
  );
}
