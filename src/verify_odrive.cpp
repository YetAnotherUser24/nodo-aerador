/**
 * @file verify_odrive.cpp
 * @brief ODrive telemetry verification firmware for env:esp32-s3-verify
 *
 * Reads 6 variables every 500ms via UART ASCII and prints them as CSV.
 * Compare side-by-side with:  python tools/odrive_monitor.py --csv
 *
 * Format:
 *   rpm:<val> RPM, vbus:<val> V, ibus:<val> A, power:<val> W, torque:<val> Nm, iq:<val> A
 *
 * Wiring (ESP32-S3 default, matches board.h):
 *   ODrive TX -> GPIO 3 (RX)
 *   ODrive RX -> GPIO 2 (TX)
 *   Common GND mandatory
 */

#include <Arduino.h>
#include "odrive_uart.h"
#include "config.h"

#if CONFIG_IDF_TARGET_ESP32S3
static HardwareSerial ODriveSerial(1);
#else
static HardwareSerial ODriveSerial(2);
#endif

static ODriveUART odrive(ODriveSerial, ODRIVE_SERIAL_RX_PIN, ODRIVE_SERIAL_TX_PIN, ODRIVE_BAUD_RATE);

void setup() {
  Serial.begin(115200);
  delay(1500);  // wait for USB CDC to enumerate
  Serial.println("ODrive Verification Firmware -- env:esp32-s3-verify");
  Serial.printf("UART pins: RX=%d TX=%d @ %lu baud\n",
                ODRIVE_SERIAL_RX_PIN, ODRIVE_SERIAL_TX_PIN, (unsigned long)ODRIVE_BAUD_RATE);
  Serial.println("Format: rpm:<val> RPM, vbus:<val> V, ibus:<val> A, power:<val> W, torque:<val> Nm, iq:<val> A");
  Serial.println();

  odrive.begin();
  delay(200);
}

void loop() {
  static unsigned long lastMs = 0;
  unsigned long now = millis();
  if (now - lastMs < 500) {
    delay(10);
    return;
  }
  lastMs = now;

  float vel   = 0.0f;
  float vbus  = 0.0f;
  float ibus  = 0.0f;
  float torq  = 0.0f;
  float iq    = 0.0f;

  // Each readFloatProperty flushes RX, sends the command, waits for reply.
  // Timeout 30ms each -- generous to avoid false negatives.
  bool ok_vel  = odrive.readFloatProperty("axis0.vel_estimate",          vel,  30);
  bool ok_vbus = odrive.readFloatProperty("vbus_voltage",                vbus, 30);
  bool ok_ibus = odrive.readFloatProperty("ibus",                        ibus, 30);
  bool ok_torq = odrive.readFloatProperty("axis0.motor.torque_estimate", torq, 30);
  bool ok_iq   = odrive.readFloatProperty("axis0.motor.foc.Iq_measured", iq,   30);

  float rpm     = ok_vel  ? vel  * 60.0f  : NAN;
  float power_w = (ok_vbus && ok_ibus) ? vbus * ibus : NAN;

  // Format: field:value unit
  Serial.printf("rpm:%.2f RPM, vbus:%.3f V, ibus:%.4f A, power:%.3f W, torque:%.4f Nm, iq:%.4f A\n",
                ok_vel  ? rpm     : NAN,
                ok_vbus ? vbus    : NAN,
                ok_ibus ? ibus    : NAN,
                (ok_vbus && ok_ibus) ? power_w : NAN,
                ok_torq ? torq    : NAN,
                ok_iq   ? iq      : NAN);
}
