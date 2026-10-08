/**
 * @file odrive_sd_logger.cpp
 * @brief Implementation of asynchronous MicroSD logger for ODrive telemetry.
 */

#include "odrive_sd_logger.h"
#include "board.h"
#include "config.h"

#if defined(SD_LOG_ENABLED) && SD_LOG_ENABLED

#include <SPI.h>
#include <SD.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

static QueueHandle_t s_logQueue = nullptr;
static TaskHandle_t s_sdTaskHandle = nullptr;
static bool s_sdReady = false;
static volatile bool s_sdPaused = false;

#if defined(SD_SPI_HOST)
static SPIClass s_sdSpi(SD_SPI_HOST);
#else
static SPIClass s_sdSpi(HSPI);
#endif

static char s_lineBuffer[128];
static char s_batchBuffer[1500];

/** Nombre de archivo 8.3 FAT del experimento activo (V4). */
static char s_logPath[24] = "/ODRLOG.CSV";
/** V4 §2.1: la grabación SD solo se habilita durante ACTIVE_EXPERIMENT. */
static volatile bool s_expLogging = false;

void odriveSdLoggerSetFile(const char* filename83) {
  if (filename83 == nullptr || filename83[0] == '\0') return;

  char base[9];
  size_t k = 0;
  for (size_t i = 0; filename83[i] != '\0' && filename83[i] != '.' && k < 8; i++) {
    char c = filename83[i];
    if (c == '/' || c == '\\' || c == ' ' || c == ':') continue;
    base[k++] = (char)toupper((unsigned char)c);
  }
  base[k] = '\0';
  if (k == 0) return;

  snprintf(s_logPath, sizeof(s_logPath), "/%s.CSV", base);
  Serial.printf("[SD Log] Experiment file set to %s\n", s_logPath);
}

void odriveSdLoggerSetActive(bool active) {
  s_expLogging = active;
  Serial.printf("[SD Log] Logging %s\n", active ? "ENABLED (experiment)" : "DISABLED (idle)");
}

static void sdWriterTask(void* param) {
  size_t batchLen = 0;
  size_t samplesInBatch = 0;
  unsigned long lastFlushMs = millis();

  while (true) {
    OdriveLogSample sample;
    // Wait up to 500 ms for an incoming sample
    if (xQueueReceive(s_logQueue, &sample, pdMS_TO_TICKS(500)) == pdTRUE) {
      if (!s_expLogging) {
        continue; // V4: sin experimento activo no se formatea ni se graba
      }
      int n = snprintf(s_lineBuffer, sizeof(s_lineBuffer),
        "%lu,%llu,%.1f,%.2f,%.3f,%.3f,%.2f,%.2f,%.1f,%.3f,%u\n",
        (unsigned long)sample.t_ms,
        (unsigned long long)sample.rtc_timestamp_ms,
        sample.target_rpm,
        sample.actual_rpm,
        sample.torque_nm,
        sample.iq_a,
        sample.ibus_a,
        sample.vbus_v,
        sample.fet_temp_c,
        sample.current_do_mg_l,
        (unsigned int)sample.control_mode
      );

      if (n > 0 && (batchLen + n < sizeof(s_batchBuffer))) {
        memcpy(s_batchBuffer + batchLen, s_lineBuffer, n);
        batchLen += n;
        samplesInBatch++;
      }
    }

    // Flush batch if batch size reached or if idle time exceeds 2000 ms
    bool shouldFlush = (samplesInBatch >= SD_LOG_FLUSH_BATCH_SIZE) ||
                       (batchLen > 0 && (millis() - lastFlushMs >= 2000));

    if (shouldFlush) {
      if (s_sdReady && !s_sdPaused && s_expLogging) {
        File f = SD.open(s_logPath, FILE_APPEND);
        if (f) {
          if (f.size() == 0) {
            f.print("t_ms,rtc_timestamp_ms,target_rpm,actual_rpm,torque_nm,iq_a,ibus_a,vbus_v,fet_temp_c,do_mg_l,mode\n");
          }
          f.write((const uint8_t*)s_batchBuffer, batchLen);
          f.flush();
          f.close();
        } else {
          Serial.printf("[SD Log] Failed to open %s for append!\n", s_logPath);
        }
      }
      batchLen = 0;
      samplesInBatch = 0;
      lastFlushMs = millis();
    }
  }
}

bool odriveSdLoggerInit() {
  Serial.println("[SD Log] Initializing asynchronous MicroSD logging...");

  s_logQueue = xQueueCreate(SD_LOG_QUEUE_SIZE, sizeof(OdriveLogSample));
  if (!s_logQueue) {
    Serial.println("[SD Log] ERROR: Failed to create FreeRTOS sample queue!");
    return false;
  }

#if defined(SD_PIN_CS) && (SD_PIN_CS != -1)
  pinMode(SD_PIN_CS, OUTPUT);
  digitalWrite(SD_PIN_CS, HIGH); // Drive CS HIGH to force SPI mode clocking compliance
  delay(10);
  s_sdSpi.begin(SD_PIN_SCLK, SD_PIN_MISO, SD_PIN_MOSI, SD_PIN_CS);
  s_sdReady = SD.begin(SD_PIN_CS, s_sdSpi, 20000000);
#else
  s_sdReady = SD.begin();
#endif

  if (s_sdReady) {
    Serial.printf("[SD Log] MicroSD card mounted successfully (Type: %d, Size: %llu MB)\n",
                  SD.cardType(), SD.cardSize() / (1024 * 1024));
  } else {
    Serial.println("[SD Log] WARNING: MicroSD card mount failed! Logger queue active but logging disabled.");
  }

  // Create Core 0 background writer task with 8 KB stack
  BaseType_t res = xTaskCreatePinnedToCore(
    sdWriterTask,
    "SdLogTask",
    8192,
    nullptr,
    1,
    &s_sdTaskHandle,
    0
  );

  return (res == pdPASS);
}

bool odriveSdLoggerEnqueue(const OdriveLogSample& sample) {
  if (!s_logQueue) return false;
  // Non-blocking push (timeout = 0)
  return (xQueueSend(s_logQueue, &sample, 0) == pdTRUE);
}

bool odriveSdLoggerIsReady() {
  return s_sdReady;
}

void odriveSdLoggerPause(bool pause) {
  s_sdPaused = pause;
}

void odriveSdLoggerFlush() {
  // Handled automatically by background task timer
}

#else

// Stubs when SD logging is disabled in configuration
bool odriveSdLoggerInit() {
  Serial.println("[SD Log] MicroSD logging disabled in configuration.");
  return true;
}

bool odriveSdLoggerEnqueue(const OdriveLogSample& sample) {
  (void)sample;
  return false;
}

bool odriveSdLoggerIsReady() {
  return false;
}

void odriveSdLoggerFlush() {}

void odriveSdLoggerPause(bool pause) {}

void odriveSdLoggerSetFile(const char* filename83) { (void)filename83; }

void odriveSdLoggerSetActive(bool active) { (void)active; }

#endif
