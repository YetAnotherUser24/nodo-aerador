#include "cloud_worker.h"
#include "config.h"
#include "aqua_link.h"  // schema-v2 transport; expands to nothing unless AQUA_TRANSPORT_MQTT
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <lwip/sockets.h>
#include "odrive_sd_logger.h"
#include "esp_now_rx.h"
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

// Batch telemetry buffer (25 samples per push for 5Hz over 5s)
static const uint8_t TELEMETRY_BATCH_SIZE = 25;
static SharedTelemetry s_telemetryBatch[TELEMETRY_BATCH_SIZE];
static uint8_t s_batchIndex = 0;
static uint8_t s_batchCount = 0; // number of valid samples

/* ---------------- V4: identidad de experimento y Reloj Maestro ---------------- */

/** Experimento activo reportado por el orquestador; vacío => el backend lo resuelve. */
static char s_activeExperimentId[40] = "";
static portMUX_TYPE s_expMux = portMUX_INITIALIZER_UNLOCKED;

static void setActiveExperimentId(const char* id) {
  portENTER_CRITICAL(&s_expMux);
  if (id != nullptr && id[0] != '\0') {
    strncpy(s_activeExperimentId, id, sizeof(s_activeExperimentId) - 1);
    s_activeExperimentId[sizeof(s_activeExperimentId) - 1] = '\0';
  } else {
    s_activeExperimentId[0] = '\0';
  }
  portEXIT_CRITICAL(&s_expMux);
}

static void getActiveExperimentId(char* out, size_t len) {
  if (len == 0) return;
  portENTER_CRITICAL(&s_expMux);
  strncpy(out, s_activeExperimentId, len - 1);
  out[len - 1] = '\0';
  portEXIT_CRITICAL(&s_expMux);
}

/** ADR-3: extrapola epoch UTC ms desde el último paquete ESP-NOW del Reloj Maestro (od-logger). */
static uint64_t currentRtcMs() {
  LoggerSyncState sync = espNowGetLoggerState();
  if (!sync.has_valid_packet) return 0;
  return sync.rtc_timestamp_ms + (uint64_t)(millis() - sync.received_at_ms);
}

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

  StaticJsonDocument<4096> doc; // enough for 25 samples
  JsonObject root = doc.to<JsonObject>();
  char expIdBuf[40];
  getActiveExperimentId(expIdBuf, sizeof(expIdBuf));
  // Si el orquestador ya fijó un experimento se reporta explícito; de lo contrario lo resuelve Next.js.
  root["experiment_id"] = (expIdBuf[0] != '\0') ? expIdBuf : "backend_resolved";
  JsonArray samples = root.createNestedArray("payload");
  for (uint8_t i = 0; i < count; ++i) {
    JsonObject obj = samples.createNestedObject();
    float speedPercent = (batch[i].actual_rpm / MAX_MOTOR_RPM) * 100.0f;
    speedPercent = constrain(speedPercent, 0.0f, 100.0f);
    obj["is_on"] = batch[i].is_on;
    obj["speed_percent"] = round(speedPercent * 10.0f) / 10.0f;
    obj["pwm_us"] = (int)batch[i].actual_rpm;
    obj["actual_rpm"] = round(batch[i].actual_rpm * 10.0f) / 10.0f;
    obj["target_rpm"] = round(batch[i].target_rpm * 10.0f) / 10.0f;
    obj["voltage_v"] = round(batch[i].vbus_voltage * 100.0f) / 100.0f;
    obj["current_a"] = round(avgIbus * 100.0f) / 100.0f;   // averaged over batch
    obj["power_w"]   = round(avgPower * 10.0f) / 10.0f;    // averaged over batch
    obj["torque_nm"] = round(batch[i].torque_estimate * 10000.0f) / 10000.0f;
    obj["iq_a"] = round(batch[i].iq_measured * 10000.0f) / 10000.0f;
    obj["status_code"] = batch[i].status_code;
    obj["rtc_timestamp_ms"] = batch[i].rtc_timestamp_ms;
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
#if AQUA_TRANSPORT_MQTT
  // The client is unused on this path: the acknowledgement rides the MQTT session that the
  // cloud task owns, so there is no second TLS handshake to pay for either.
  (void)client;
  (void)speedPercent;
  if (cmdId.length() > 0) {
    aquaLinkAck(cmdId.c_str(), success, success ? "applied" : "rejected");
  }
#else
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

  StaticJsonDocument<200> doc;
  doc["success"] = success;
  doc["actual_speed_percent"] = speedPercent;
  // V4 §2.2: instante exacto de ejecución (epoch UTC ms) para anular la latencia de red al graficar.
  uint64_t rtc = currentRtcMs();
  if (rtc > 0) doc["rtc_timestamp_ms"] = rtc;

  String body;
  serializeJson(doc, body);
  int code = http.POST(body);
  remoteLogPrintf("[Cloud] Acknowledged cmd %s: HTTP %d\n", cmdId.c_str(), code);
  if (code < 0) {
    closeClientSocket(client);
  }

  http.end();
#endif  // AQUA_TRANSPORT_MQTT
}

/** Executes a command body. Defined below so both transports share one decision. */
static bool executeCommandBody(const String& payload, WiFiClientSecure& client);

#if !AQUA_TRANSPORT_MQTT
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
    return executeCommandBody(payload, client);
  }
  return false;
}
#endif  // !AQUA_TRANSPORT_MQTT

/**
 * Execute one command body: the action ladder, the actuator changes and the acknowledgement.
 *
 * Lifted out of the HTTP poll so the polled and pushed paths run the *same* decision. The
 * parser is deliberately unchanged -- only the source of the bytes differs.
 *
 * The inner braces stay because the original scoped the JSON document to reclaim its stack
 * before the acknowledgement starts a TLS handshake; that reason still applies on the legacy
 * path, and keeping the scope identical keeps the two transports comparable.
 */
static bool executeCommandBody(const String& payload, WiFiClientSecure& client) {
  {
    String cmdId;
    String cmdType;
    float speedPct = 0.0f;
    bool hasCmd = false;
    bool isEmergency = false;
    bool isStop = false;
    bool isThrottle = false;
    // V4: intención semántica transportada en payload.action (el CHECK de la BD limita command_type a 5 valores)
    bool isStateAction = false;
    bool isModeAction = false;
    bool isForceOn = false;
    bool isForceOff = false;
    bool isClearEstop = false;
    String stateStr, expIdStr, csvStr;
    // Receta de motor transportada por `set_state` (resuelta en el servidor desde el registro
    // de experimentos). Antes el ODrive habilitaba el log SD pero NUNCA se armaba, así que el
    // motor quedaba indefinidamente en 0 RPM al iniciar un experimento.
    String motorModeStr = "";
    float motorTargetDo = -1.0f;
    float motorThrottlePct = -1.0f;
    ControlConfig cfg = s_engine->getConfig();

    {
      StaticJsonDocument<768> doc;
      DeserializationError err = deserializeJson(doc, payload);
      if (!err && (doc["has_command"] == true || doc["has_command"].as<bool>())) {
        JsonObject cmd = doc["command"];
        cmdId = cmd["id"].as<String>();
        cmdType = cmd["command_type"].as<String>();
        speedPct = cmd["speed_percent"].as<float>();
        hasCmd = true;

        JsonObject p = cmd["payload"];
        String action = "";
        if (!p.isNull() && p["action"].is<const char*>()) action = p["action"].as<String>();
        if (action.length() == 0) action = cmdType;

        isStateAction = (action == "set_state" || action == "start_experiment" || action == "stop_experiment");
        isModeAction = (action == "set_mode" || action == "set_config" ||
                        (!p.isNull() && (p.containsKey("mode") || p.containsKey("kp") ||
                                         p.containsKey("ki") || p.containsKey("kd"))));
        isForceOn = (action == "force_on");
        isForceOff = (action == "force_off");
        isClearEstop = (action == "clear_estop" || action == "resume");

        if (isStateAction) {
          stateStr = p["state"].is<const char*>() ? p["state"].as<String>() : String("");
          expIdStr = p["experiment_id"].is<const char*>() ? p["experiment_id"].as<String>() : String("");
          csvStr   = p["csv_filename"].is<const char*>() ? p["csv_filename"].as<String>() : String("");
          if (action == "start_experiment") stateStr = "ACTIVE_EXPERIMENT";

          motorModeStr = p["motor_mode"].is<const char*>() ? p["motor_mode"].as<String>() : String("");
          if (p.containsKey("motor_target_do")) motorTargetDo = p["motor_target_do"].as<float>();
          if (p.containsKey("motor_throttle_pct")) motorThrottlePct = p["motor_throttle_pct"].as<float>();
        } else if (isModeAction) {
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
        } else if (action == "emergency_stop" || cmdType == "emergency_stop") {
          isEmergency = true;
        } else if (action == "stop" || cmdType == "stop") {
          isStop = true;
        } else if (action == "start" || action == "set_speed" || cmdType == "start" || cmdType == "set_speed") {
          isThrottle = true;
          if ((action == "start" || cmdType == "start") && speedPct <= 0.0f) {
            speedPct = (cfg.manual_throttle_pct > 0.0f) ? cfg.manual_throttle_pct : 20.0f;
          }
        }
      } else if (err) {
        remoteLogPrintf("[Cloud] Command JSON parse error: %s\n", err.c_str());
      }
    } // doc is destroyed here, reclaiming 512B stack before acknowledgeCommand starts TLS

    if (hasCmd) {
      remoteLogPrintf("[Cloud] Executing command '%s' (speed: %.1f%%)\n", cmdType.c_str(), speedPct);

      if (isStateAction) {
        // Máquina de estados del orquestador: IDLE | ACTIVE_EXPERIMENT | MANUAL_OVERRIDE
        bool activate = (stateStr == "ACTIVE_EXPERIMENT");
        if (activate) {
          setActiveExperimentId(expIdStr.c_str());
          if (csvStr.length() > 0) odriveSdLoggerSetFile(csvStr.c_str());
          odriveSdLoggerSetActive(true);
          s_engine->setExperimentLogging(true);
          remoteLogPrintf("[Cloud] Experiment logging ENABLED (id=%s)\n", expIdStr.c_str());
        } else {
          s_engine->setExperimentLogging(false);
          odriveSdLoggerSetActive(false);
          setActiveExperimentId("");
          remoteLogPrintf("[Cloud] Experiment logging DISABLED (state=%s)\n", stateStr.c_str());
        }

        // Armado del actuador. Se usa `configure()` y no `setManualThrottle()` a propósito:
        // `setManualThrottle()` limpia el latch de emergencia, y arrancar un experimento NO debe
        // liberar un E-Stop de forma implícita. Si el latch está activo el motor permanece en 0
        // (seguro) y hay que liberarlo de forma explícita con `clear_estop` (botón REANUDAR).
        if (activate && motorModeStr == "pid") {
          ControlConfig m = s_engine->getConfig();
          m.mode = MODE_PID;
          if (motorTargetDo >= 0.0f) m.target_do_mg_l = motorTargetDo;
          s_engine->configure(m);
          savePersistedConfig(m);
          remoteLogPrintf("[Cloud] Motor ARMED in PID (setpoint=%.3f mg/L)\n",
                          m.target_do_mg_l);
        } else if (activate && motorModeStr == "manual") {
          ControlConfig m = s_engine->getConfig();
          m.mode = MODE_MANUAL;
          if (motorThrottlePct >= 0.0f) m.manual_throttle_pct = constrain(motorThrottlePct, 0.0f, 100.0f);
          s_engine->configure(m);
          savePersistedConfig(m);
          remoteLogPrintf("[Cloud] Motor ARMED in MANUAL (%.1f%% -> ~%.0f RPM)\n",
                          m.manual_throttle_pct, (m.manual_throttle_pct / 100.0f) * m.max_rpm);
        } else {
          // IDLE, MANUAL_OVERRIDE o receta sin actuador: actuadores desactivados (ADD 2.1/2.3).
          ControlConfig m = s_engine->getConfig();
          m.mode = MODE_MANUAL;
          m.manual_throttle_pct = 0.0f;
          s_engine->configure(m);
          if (!activate) remoteLogPrintf("[Cloud] Motor set to 0 RPM (state=%s)\n", stateStr.c_str());
        }

        acknowledgeCommand(client, cmdId, true, 0.0f);
      } else if (isModeAction) {
        s_engine->configure(cfg);
        savePersistedConfig(cfg);
        acknowledgeCommand(client, cmdId, true, speedPct);
      } else if (isForceOn) {
        float pct = (cfg.manual_throttle_pct > 0.0f) ? cfg.manual_throttle_pct : 50.0f;
        s_engine->setManualThrottle(pct);
        acknowledgeCommand(client, cmdId, true, pct);
      } else if (isForceOff) {
        s_engine->setManualThrottle(0.0f);
        acknowledgeCommand(client, cmdId, true, 0.0f);
      } else if (isClearEstop) {
        // Única vía explícita de liberar el latch de emergencia (botón REANUDAR del dashboard).
        s_engine->clearEmergencyStop();
        remoteLogPrintf("[Cloud] Emergency stop latch CLEARED by operator\n");
        acknowledgeCommand(client, cmdId, true, 0.0f);
      } else if (isEmergency) {
        s_engine->triggerEmergencyStop();
        acknowledgeCommand(client, cmdId, true, 0.0f);
      } else if (isStop) {
        s_engine->setManualThrottle(0.0f);
        acknowledgeCommand(client, cmdId, true, 0.0f);
      } else if (isThrottle) {
        s_engine->setManualThrottle(speedPct);
        acknowledgeCommand(client, cmdId, true, speedPct);
      } else if (cmdType == "reboot") {
        acknowledgeCommand(client, cmdId, true, 0.0f);
        delay(500);
        ESP.restart();
      } else {
        // Nothing in the ladder matched.
        //
        // This branch did not exist: an unrecognised action fell off the end of the chain with
        // no branch taken and *nothing* acknowledged, so the gateway held the command open
        // until its deadline expired and the operator saw a command that never resolved. A node
        // that cannot carry out a verb should say so -- silence is indistinguishable from a
        // node that is offline, and it is the harder of the two to diagnose.
        remoteLogPrintf("[Cloud] Unsupported command action '%s'; rejecting\n", cmdType.c_str());
        acknowledgeCommand(client, cmdId, false, 0.0f);
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
      
      // If auto-reconnect fails for 30s, force a clean restart of the WiFi stack.
      // NOTA: se preserva el modo STA (disconnect(false,false)) para no derribar la capa
      // ESP-NOW, de la que depende el lazo de control de seguridad (invariante del sistema).
      if (millis() - lastWifiRetry > 30000) {
        Serial.printf("[Cloud Worker] WiFi stuck. Forcing restart to '%s'...\n", WIFI_SSID);
        WiFi.mode(WIFI_STA);
        WiFi.disconnect(false, false);
        vTaskDelay(pdMS_TO_TICKS(100));
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        lastWifiRetry = millis();
      }
    } else {
      if (!wasConnected) {
        Serial.printf("[Cloud Worker] WiFi connected. IP: %s (RSSI: %d dBm, ch: %u)\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI(), (unsigned)WiFi.channel());
        // Anchor the system clock. The node's *intended* time authority is the DS3231 on the OD
        // logger, relayed over ESP-NOW, which is why nothing here ever asked NTP for the time --
        // but that source only exists when the logger is in radio range, and on the bench
        // (virtual mode) it never is. Without a fallback every telemetry sample stamps 0, so the
        // per-sample timestamps the envelope carries all collapse onto one instant.
        configTime(0, 0, "pool.ntp.org", "time.nist.gov");
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

#if AQUA_TRANSPORT_MQTT
      // Serviced here rather than at the top of the task: the loop `continue`s before this
      // point while WiFi is down, and there is no point driving an MQTT client without a
      // network. This is also the only task that touches the transport, which is what the
      // module's thread-safety contract requires.
      aquaLinkLoop();
#endif

      if (now - lastTelemSample >= TELEMETRY_SAMPLE_INTERVAL_MS) {
        lastTelemSample = now;
        SharedTelemetry telem = getSharedTelemetry();
#if AQUA_TRANSPORT_MQTT
        // Queued the moment it is taken, carrying its OWN instant. A batch timestamped once on
        // the way out puts every sample on one millisecond and a plot draws one point where it
        // was given twenty-five.
        // The node's own time authority is the DS3231 on the OD logger, relayed over ESP-NOW.
        // Fall back to the system clock, which NTP anchors on connect. Both are needed: with an
        // unanchored clock every sample stamps 0 and the batch collapses onto one instant, which
        // is the exact failure per-sample timestamps exist to prevent.
        // ESP-NOW from the logger first, since it is the deployment's persistent authority, then
        // the transport's best-known clock -- NTP when there is internet, the retained
        // `aqua/time` broadcast when there is not. Falling back to `millis()` last keeps the
        // samples ordered and flagged monotonic, which beats collapsing them all onto zero.
        uint64_t sampleMs = telem.rtc_timestamp_ms;
        if (sampleMs == 0) sampleMs = aquaLinkNowMs();
        if (sampleMs == 0) sampleMs = (uint64_t)millis();
        aquaLinkQueue(telem, sampleMs);
#else
        s_telemetryBatch[s_batchIndex] = telem;
        s_batchIndex = (s_batchIndex + 1) % TELEMETRY_BATCH_SIZE;
        if (s_batchCount < TELEMETRY_BATCH_SIZE) s_batchCount++;
#endif
      }

#if !AQUA_TRANSPORT_MQTT
      // Priority 1: Check cloud commands (dynamic interval: fast in Manual mode).
      //
      // With MQTT on, commands are pushed and this poll is compiled out. On this node it was
      // the heaviest thing Core 0 did: a TLS handshake every 1.5 s, down to 1 s in manual mode.
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
#endif  // !AQUA_TRANSPORT_MQTT

      // Priority 2: Periodic batch telemetry push
      if (now - lastTelemPush >= TELEMETRY_PUSH_INTERVAL_MS) {
        lastTelemPush = now;
#if AQUA_TRANSPORT_MQTT
        // One envelope per push interval, holding every sample taken since the last one. On
        // failure the batch is retained, so an outage costs latency rather than data.
        aquaLinkFlush();
#else
        // Send batch if we have at least one sample (send whatever is collected)
        if (s_batchCount > 0) {
          sendTelemetryBatch(client, s_telemetryBatch, s_batchCount);
          // Reset batch after sending
          s_batchCount = 0;
          s_batchIndex = 0;
        }
#endif
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

#if AQUA_TRANSPORT_MQTT
/**
 * Translate a schema-v2 command into the shape the shared executor already understands.
 *
 * The v2 message carries `cmd_id`, `action`, `target` and `exp`. The legacy envelope nested the
 * same information under `command:{id, command_type, payload:{...}}`. Only the shape differs, so
 * this reshapes rather than re-deciding anything — `executeCommandBody` remains the one place
 * that knows what `set_state` or `clear_estop` mean.
 *
 * A command with no action is dropped rather than acknowledged: there is nothing to carry out,
 * and acking it would tell the gateway the node acted on an empty verb.
 */
void aeratorHandleMqttCommand(const char* payload, size_t length) {
  StaticJsonDocument<1024> envelope;
  if (deserializeJson(envelope, payload, length) != DeserializationError::Ok) {
    remoteLogPrintf("[AQUA] command payload was not valid JSON; ignored\n");
    return;
  }

  const char* cmdId = envelope["cmd_id"].is<const char*>() ? envelope["cmd_id"].as<const char*>() : "";
  const char* action = envelope["action"].is<const char*>() ? envelope["action"].as<const char*>() : "";
  if (action[0] == '\0') {
    remoteLogPrintf("[AQUA] command had no action; ignored\n");
    return;
  }

  // Attribution travels with the command, and is cleared only by the actions that *end* a run.
  // A `start_experiment` that does not repeat the id keeps the current one, because a UI sending
  // a start has no reason to restate the run it already began.
  const char* exp = envelope["exp"].is<const char*>() ? envelope["exp"].as<const char*>() : nullptr;
  const String stateArg = envelope["target"]["state"].is<const char*>()
                              ? envelope["target"]["state"].as<String>()
                              : String("");
  const bool endsRun = String(action) == "stop_experiment" ||
                       (String(action) == "set_state" && stateArg != "ACTIVE_EXPERIMENT" &&
                        stateArg != "");
  if (endsRun) {
    setActiveExperimentId("");
  } else if (exp != nullptr && exp[0] != '\0') {
    setActiveExperimentId(exp);
  }

  // Rebuild the legacy envelope the executor reads. Synthesising it here rather than teaching
  // the executor a second shape is what keeps one parser instead of two that drift.
  StaticJsonDocument<1024> legacy;
  JsonObject root = legacy.to<JsonObject>();
  root["has_command"] = true;
  JsonObject cmd = root.createNestedObject("command");
  cmd["id"] = cmdId;
  cmd["command_type"] = action;
  JsonObject params = cmd.createNestedObject("payload");
  JsonObjectConst target = envelope["target"].as<JsonObjectConst>();
  for (JsonPairConst kv : target) {
    params[kv.key()] = kv.value();
  }
  params["action"] = action;
  if (envelope["target"]["speed_percent"].is<float>()) {
    cmd["speed_percent"] = envelope["target"]["speed_percent"].as<float>();
  }

  String body;
  serializeJson(legacy, body);
  // The executor only reaches for the client on the legacy path; under MQTT the acknowledgement
  // rides the session the cloud task already owns, so this one is never opened.
  static WiFiClientSecure unusedClient;
  executeCommandBody(body, unusedClient);
}
#endif  // AQUA_TRANSPORT_MQTT
