#include "cloud_worker.h"
#include "config.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <lwip/sockets.h>
#include "odrive_sd_logger.h"
#if defined(OTA_ENABLED) && OTA_ENABLED
#include <ArduinoOTA.h>
#endif

#ifndef LED_BUILTIN
#define LED_BUILTIN 8 // GPIO 8 is the blue LED on ESP32-S3 SuperMini
#endif

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

#if defined(TELNET_LOG_ENABLED) && TELNET_LOG_ENABLED
static int s_telnetServerFd = -1;
static int s_telnetClientFd = -1;
static portMUX_TYPE s_telnetMux = portMUX_INITIALIZER_UNLOCKED;

static void setupTelnet() {
  if (s_telnetServerFd >= 0) return;
  s_telnetServerFd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (s_telnetServerFd < 0) return;

  int enable = 1;
  setsockopt(s_telnetServerFd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));

  int flags = fcntl(s_telnetServerFd, F_GETFL, 0);
  fcntl(s_telnetServerFd, F_SETFL, flags | O_NONBLOCK);

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(TELNET_LOG_PORT);

  if (bind(s_telnetServerFd, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
    close(s_telnetServerFd);
    s_telnetServerFd = -1;
    return;
  }
  listen(s_telnetServerFd, 1);
  Serial.printf("[Telnet] Remote terminal active: telnet://%s:%d\n", WiFi.localIP().toString().c_str(), TELNET_LOG_PORT);
}

static void handleTelnetClients() {
  if (s_telnetServerFd < 0) return;

  struct sockaddr_in clientAddr;
  socklen_t clientLen = sizeof(clientAddr);
  int newFd = accept(s_telnetServerFd, (struct sockaddr*)&clientAddr, &clientLen);
  if (newFd >= 0) {
    int nodelay = 1;
    setsockopt(newFd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
    int flags = fcntl(newFd, F_GETFL, 0);
    fcntl(newFd, F_SETFL, flags | O_NONBLOCK);

    portENTER_CRITICAL(&s_telnetMux);
    int oldFd = s_telnetClientFd;
    s_telnetClientFd = newFd;
    portEXIT_CRITICAL(&s_telnetMux);

    if (oldFd >= 0) {
      close(oldFd);
    }

    const char* banner = "\r\n=======================================================\r\n  AquaControl: ESP32 Remote Telnet Console Connected  \r\n=======================================================\r\n\r\n";
    send(newFd, banner, strlen(banner), 0);
  }
}

void remoteLogPrint(const char* str) {
  Serial.print(str);

  portENTER_CRITICAL(&s_telnetMux);
  int fd = s_telnetClientFd;
  portEXIT_CRITICAL(&s_telnetMux);

  if (fd >= 0) {
    size_t len = strlen(str);
    int sent = send(fd, str, len, 0);
    if (sent < 0 && (errno == ECONNRESET || errno == EPIPE || errno == ENOTCONN || errno == EBADF)) {
      portENTER_CRITICAL(&s_telnetMux);
      if (s_telnetClientFd == fd) {
        s_telnetClientFd = -1;
      }
      portEXIT_CRITICAL(&s_telnetMux);
      close(fd);
    }
  }
}

void remoteLogPrintf(const char* format, ...) {
  char buf[384];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  remoteLogPrint(buf);
}
#else
void remoteLogPrint(const char* str) {
  Serial.print(str);
}

void remoteLogPrintf(const char* format, ...) {
  char buf[384];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  Serial.print(buf);
}
#endif

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
  remoteLogPrintf("[NVS] Loaded config: mode=%d | target_do=%.2f | kp=%.1f ki=%.2f kd=%.1f\n",
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
  remoteLogPrintf("[NVS] Persisted updated control parameters to Flash.\n");
}

static void closeClientSocket(WiFiClientSecure& client) {
  client.stop();
}

// Single telemetry send (kept for compatibility, not used in batch mode)
static void sendTelemetry(const SharedTelemetry& telem) {
  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(4);
  HTTPClient http;
  String url = String("https://") + AQUACONTROL_HOST + AQUACONTROL_MOTOR_PATH;
  http.setReuse(false);
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
  doc["torque_nm"] = round(telem.torque_estimate * 10000.0f) / 10000.0f;
  doc["iq_a"] = round(telem.iq_measured * 10000.0f) / 10000.0f;
  doc["status_code"] = telem.status_code;

  String body;
  serializeJson(doc, body);
  int code = http.POST(body);
  if (code >= 200 && code < 300) {
    remoteLogPrintf("[Cloud] Telemetry sent: %.0f RPM | %.1fV %.2fA (HTTP %d)\n",
                  telem.actual_rpm, telem.vbus_voltage, telem.ibus_current, code);
  } else {
    remoteLogPrintf("[Cloud] Telemetry POST returned %d: %s\n", code, http.errorToString(code).c_str());
  }
  http.end();
  client.stop();
}

// Batch telemetry send – sends an array of telemetry samples
static void sendTelemetryBatch(WiFiClientSecure& client, const SharedTelemetry* batch, uint8_t count) {
  if (count == 0) return; // nothing to send
  HTTPClient http;
  String url = String("https://") + AQUACONTROL_HOST + AQUACONTROL_MOTOR_PATH;
  http.setReuse(true);
  if (!http.begin(client, url)) {
    remoteLogPrintf("[Cloud] Telemetry batch begin failed\n");
    closeClientSocket(client);
    return;
  }
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Connection", "keep-alive");
  http.addHeader("X-Device-Key", AQUACONTROL_DEVICE_KEY);
  http.setTimeout(4000);

  // Build JSON array of samples
  // Average the noisy DC-bus measurements (ibus, power) over all samples to smooth PWM switching noise.
  float avgIbus = 0.0f, avgPower = 0.0f;
  for (uint8_t i = 0; i < count; ++i) {
    avgIbus  += batch[i].ibus_current;
    avgPower += batch[i].power_watts;
  }
  avgIbus  /= count;
  avgPower /= count;

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
    obj["current_a"] = round(avgIbus * 100.0f) / 100.0f;   // averaged over batch
    obj["power_w"]   = round(avgPower * 10.0f) / 10.0f;    // averaged over batch
    obj["torque_nm"] = round(batch[i].torque_estimate * 10000.0f) / 10000.0f;
    obj["iq_a"] = round(batch[i].iq_measured * 10000.0f) / 10000.0f;
    obj["status_code"] = batch[i].status_code;
  }

  String body;
  serializeJson(doc, body);
  int code = http.POST(body);
  if (code >= 200 && code < 300) {
    remoteLogPrintf("[Cloud] Batch telemetry (%d samples) sent (HTTP %d)\n", count, code);
  } else {
    remoteLogPrintf("[Cloud] Batch telemetry POST error %d: %s\n", code, http.errorToString(code).c_str());
    if (code < 0) {
      closeClientSocket(client);
    }
  }
  http.end();
}

static void acknowledgeCommand(WiFiClientSecure& client, const String& cmdId, bool success, float speedPercent) {
  HTTPClient http;
  String url = String("https://") + AQUACONTROL_HOST + AQUACONTROL_ACK_PATH + "/" + cmdId + "/acknowledge";
  http.setReuse(true);

  if (!http.begin(client, url)) {
    remoteLogPrintf("[Cloud] Acknowledge begin failed\n");
    closeClientSocket(client);
    return;
  }

  http.addHeader("Content-Type", "application/json");
  http.addHeader("Connection", "keep-alive");
  http.addHeader("X-Device-Key", AQUACONTROL_DEVICE_KEY);
  http.setTimeout(4000);

  StaticJsonDocument<150> doc;
  doc["success"] = success;
  doc["actual_speed_percent"] = speedPercent;

  String body;
  serializeJson(doc, body);
  int code = http.POST(body);
  remoteLogPrintf("[Cloud] Acknowledged cmd %s: HTTP %d\n", cmdId.c_str(), code);
  if (code < 0) {
    closeClientSocket(client);
  }

  http.end();
}

static bool pollPendingCommands(WiFiClientSecure& client) {
  HTTPClient http;
  String url = String("https://") + AQUACONTROL_HOST + AQUACONTROL_PENDING_CMD + "?device_key=" + AQUACONTROL_DEVICE_KEY;
  http.setReuse(true);

  if (!http.begin(client, url)) {
    remoteLogPrintf("[Cloud] Poll begin failed\n");
    closeClientSocket(client);
    return false;
  }

  http.addHeader("Connection", "keep-alive");
  http.setTimeout(4000);

  int code = http.GET();
  String payload;
  if (code == 200) {
    payload = http.getString();
  } else if (code < 0) {
    remoteLogPrintf("[Cloud] Poll connection error: %s (%d)\n", http.errorToString(code).c_str(), code);
  } else if (code != 200) {
    remoteLogPrintf("[Cloud] Poll returned HTTP %d\n", code);
  }

  // End HTTP poll session before processing actions or sending acknowledge POST
  http.end();
  if (code < 0) {
    closeClientSocket(client);
  }

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
        remoteLogPrintf("[Cloud] Command JSON parse error: %s\n", err.c_str());
      }
    } // doc is destroyed here, reclaiming 512B stack before acknowledgeCommand starts TLS

    if (hasCmd) {
      remoteLogPrintf("[Cloud] Executing command '%s' (speed: %.1f%%)\n", cmdType.c_str(), speedPct);

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

#if defined(OTA_ENABLED) && OTA_ENABLED
static volatile bool s_isOtaUpdating = false;

static void setupOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPort(OTA_PORT);
  if (strlen(OTA_PASSWORD) > 0) {
    ArduinoOTA.setPassword(OTA_PASSWORD);
  }

  ArduinoOTA.onStart([]() {
    s_isOtaUpdating = true;
    odriveSdLoggerPause(true); // Stop SD writes to free CPU and SPI bus for OTA
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    remoteLogPrintf("[OTA] Update started (%s). Disarming motor for safety & dedicating CPU to OTA...\n", type.c_str());
    if (s_engine) {
      s_engine->triggerEmergencyStop();
    }
  });

  ArduinoOTA.onEnd([]() {
    s_isOtaUpdating = false;
    odriveSdLoggerPause(false);
    remoteLogPrintf("\n[OTA] Update completed successfully. Rebooting...\n");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static unsigned int lastPercent = 0;
    unsigned int percent = (progress / (total / 100));
    if (percent != lastPercent && percent % 10 == 0) {
      remoteLogPrintf("[OTA] Progress: %u%%\n", percent);
      lastPercent = percent;
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    s_isOtaUpdating = false;
    odriveSdLoggerPause(false);
    remoteLogPrintf("[OTA] Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) remoteLogPrintf("Auth Failed\n");
    else if (error == OTA_BEGIN_ERROR) remoteLogPrintf("Begin Failed\n");
    else if (error == OTA_CONNECT_ERROR) remoteLogPrintf("Connect Failed\n");
    else if (error == OTA_RECEIVE_ERROR) remoteLogPrintf("Receive Failed\n");
    else if (error == OTA_END_ERROR) remoteLogPrintf("End Failed\n");
  });

  ArduinoOTA.begin();
  remoteLogPrintf("[OTA] ArduinoOTA service ready: %s.local:%d\n", OTA_HOSTNAME, OTA_PORT);
}
#endif

static void cloudWorkerTask(void* pvParameters) {
  remoteLogPrintf("[Cloud Worker] Task started on Core 0\n");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Assuming active-low LED, turn OFF initially

  WiFiClientSecure client;
  client.setInsecure(); // ponytail: skip cert pinning for serverless Vercel endpoints
  client.setTimeout(4);
  client.setHandshakeTimeout(5);

  unsigned long lastTelemPush = 0;
  unsigned long lastTelemSample = 0;
  unsigned long lastCmdPoll = 0;
  uint8_t fastPollBurst = 0;
#if defined(OTA_ENABLED) && OTA_ENABLED
  bool otaInitialized = false;
#endif
#if defined(TELNET_LOG_ENABLED) && TELNET_LOG_ENABLED
  bool telnetInitialized = false;
#endif

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setTxPower(WIFI_POWER_8_5dBm); // Reduce TX power for ESP32-S3 SuperMini stability
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long lastWifiRetry = millis();
  bool wasConnected = false;

  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      if (wasConnected) {
        Serial.println("[Cloud Worker] WiFi disconnected. Auto-reconnect active...");
        digitalWrite(LED_BUILTIN, HIGH); // Turn OFF LED
        wasConnected = false;
        lastWifiRetry = millis();
      }
      
      // If auto-reconnect fails for 30s, force a clean restart of the WiFi stack
      if (millis() - lastWifiRetry > 30000) {
        Serial.printf("[Cloud Worker] WiFi stuck. Forcing restart to '%s'...\n", WIFI_SSID);
        WiFi.disconnect(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        lastWifiRetry = millis();
      }
    } else {
      if (!wasConnected) {
        Serial.printf("[Cloud Worker] WiFi connected. IP: %s (RSSI: %d dBm)\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
        digitalWrite(LED_BUILTIN, LOW); // Turn ON LED
        wasConnected = true;
      }
    }

    if (WiFi.status() == WL_CONNECTED) {
#if defined(OTA_ENABLED) && OTA_ENABLED
      if (!otaInitialized) {
        setupOTA();
        otaInitialized = true;
      }
      ArduinoOTA.handle();

      if (s_isOtaUpdating) {
        if (client.connected()) {
          client.stop();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;
      }
#endif

#if defined(TELNET_LOG_ENABLED) && TELNET_LOG_ENABLED
      if (!telnetInitialized) {
        setupTelnet();
        telnetInitialized = true;
      }
      handleTelnetClients();
#endif

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
