/**
 * @file odrive_sd_logger.h
 * @brief Asynchronous, non-blocking MicroSD logger for ODrive motor & process telemetry.
 *
 * Architecture:
 *   - Core 1 pushes samples into a FreeRTOS RAM queue in < 5 µs (zero control-loop stall).
 *   - Core 0 runs an asynchronous background task that drains the queue, batches lines,
 *     and appends them to a CSV file on the dedicated HSPI MicroSD slot.
 */

#pragma once

#include <Arduino.h>

struct OdriveLogSample {
  uint32_t t_ms;           // Controller local millis()
  uint64_t rtc_timestamp_ms; // DS3231 RTC ms from od-logger (or 0 if pending)
  float target_rpm;        // Commanded target speed
  float actual_rpm;        // Measured motor speed
  float torque_nm;         // Estimated mechanical torque
  float iq_a;              // Phase current (Iq)
  float ibus_a;            // Total DC bus current
  float vbus_v;            // DC bus power supply voltage
  float fet_temp_c;        // Inverter MOSFET temperature
  float current_do_mg_l;   // Synchronized Dissolved Oxygen reading
  uint8_t control_mode;    // 0=PID, 1=Fuzzy, 2=Manual
};

/**
 * @brief Initializes the MicroSD card over HSPI and spawns the Core 0 writer task.
 * @return True if initialized (or gracefully disabled), false on critical allocation error.
 */
bool odriveSdLoggerInit();

/**
 * @brief Enqueues a sample from Core 1.
 * Non-blocking: returns immediately without waiting for SD write or flash erase.
 * @param sample Telemetry sample snapshot.
 * @return True if enqueued, false if queue is full (sample dropped to protect control loop).
 */
bool odriveSdLoggerEnqueue(const OdriveLogSample& sample);

/**
 * @brief Checks whether the SD logger is physically ready and mounted.
 */
bool odriveSdLoggerIsReady();

/**
 * @brief Explicitly flushes any pending samples to the SD card.
 */
void odriveSdLoggerFlush();

/**
 * @brief Pauses or resumes the SD logger (e.g. during OTA updates to prevent SPI bus blocking).
 */
void odriveSdLoggerPause(bool pause);

/**
 * @brief Sets the 8.3 FAT log filename for the active experiment (e.g. "EXP_01.CSV").
 * The basename is sanitized to at most 8 uppercase characters plus the ".CSV" extension.
 */
void odriveSdLoggerSetFile(const char* filename83);

/**
 * @brief Enables/disables experiment-scoped logging (V4 §2.1: logging is disabled in IDLE).
 */
void odriveSdLoggerSetActive(bool active);
