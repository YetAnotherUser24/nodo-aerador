/**
 * @file board.h
 * @brief Board pin definitions and hardware isolation for AquaControl odrive-controller.
 *
 * Supported Boards:
 *   - BOARD_LILYGO_LORA32 : LilyGO T3 LoRa32 (v1.6) + Custom Instrumentation HAT
 *   - Generic ESP32 / S3  : Default fallback when no board flag is specified
 */

#pragma once

#include <Arduino.h>

/* ==========================================================================
 *  BOARD: LilyGO T3 LoRa32 + Custom Instrumentation HAT
 * ========================================================================== */
#if defined(BOARD_LILYGO_LORA32)

#define BOARD_NAME "LilyGO T3 LoRa32 + HAT"

#ifndef LED_PIN
#define LED_PIN 25
#endif

#ifndef WAKEUP_PIN
#define WAKEUP_PIN GPIO_NUM_34
#endif

/* Power Enable Control Pins */
#ifndef EN_5V_PIN
#define EN_5V_PIN -1
#endif
#ifndef EN_5V_ACTIVE_HIGH
#define EN_5V_ACTIVE_HIGH true
#endif

/* LoRa SX1278 Dedicated SPI Controller (VSPI / SPI3) */
#ifndef LORA_SPI_HOST
#define LORA_SPI_HOST VSPI
#endif

#ifndef LORA_CS_PIN
#define LORA_CS_PIN 18
#endif

#ifndef LORA_PIN_SCLK
#define LORA_PIN_SCLK 5
#endif

#ifndef LORA_PIN_MISO
#define LORA_PIN_MISO 19
#endif

#ifndef LORA_PIN_MOSI
#define LORA_PIN_MOSI 27
#endif

#ifndef LORA_RST_PIN
#define LORA_RST_PIN 23
#endif

#ifndef LORA_DIO0_PIN
#define LORA_DIO0_PIN 26
#endif

/* Main I2C Bus Pins (DS3231MZ + OLED SSD1306) */
#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN 21
#endif
#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN 22
#endif

/* OLED Display Configuration (0.96" SSD1306 onboard) */
#ifndef OLED_CONTROLLER_SSD1306
#define OLED_CONTROLLER_SSD1306
#endif
#ifndef OLED_COLUMN_OFFSET
#define OLED_COLUMN_OFFSET 0x00
#endif
#ifndef OLED_TYPE_NAME
#define OLED_TYPE_NAME "0.96\" SSD1306"
#endif

/* Hardware UART for ODrive S1 on HAT */
#ifndef ODRIVE_SERIAL_RX_PIN
#define ODRIVE_SERIAL_RX_PIN 39
#endif
#ifndef ODRIVE_SERIAL_TX_PIN
#define ODRIVE_SERIAL_TX_PIN 4
#endif

/* Onboard MicroSD Dedicated SPI Controller (HSPI / SPI2) */
#ifndef SD_SPI_HOST
#define SD_SPI_HOST HSPI
#endif

#ifndef SD_PIN_CS
#define SD_PIN_CS 13
#endif

#ifndef SD_PIN_SCLK
#define SD_PIN_SCLK 14
#endif

#ifndef SD_PIN_MISO
#define SD_PIN_MISO 2
#endif

#ifndef SD_PIN_MOSI
#define SD_PIN_MOSI 15
#endif

/* RTC DS3231MZ Interrupt Pin */
#ifndef RTC_INT_PIN
#define RTC_INT_PIN 34
#endif

/* Battery ADC Pin */
#ifndef BOARD_BAT_ADC_PIN
#define BOARD_BAT_ADC_PIN 36
#endif

#else

/* ==========================================================================
 *  DEFAULT: Generic ESP32 / ESP32-S3 DevKit
 * ========================================================================== */
#ifndef BOARD_NAME
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define BOARD_NAME "ESP32-S3 SuperMini / DevKit"
#else
#define BOARD_NAME "Generic ESP32 DevKit"
#endif
#endif

#ifndef LED_PIN
#define LED_PIN -1
#endif

/* Hardware UART for ODrive S1 */
#ifndef ODRIVE_SERIAL_TX_PIN
#define ODRIVE_SERIAL_TX_PIN 2
#endif

#ifndef ODRIVE_SERIAL_RX_PIN
#define ODRIVE_SERIAL_RX_PIN 3
#endif

/* Battery ADC Pin */
#ifndef BOARD_BAT_ADC_PIN
#define BOARD_BAT_ADC_PIN 13
#endif


/* Onboard MicroSD Dedicated SPI Controller (HSPI / SPI2) */
#ifndef SD_SPI_HOST
#define SD_SPI_HOST HSPI
#endif

#ifndef SD_PIN_CS
#define SD_PIN_CS 7 
#endif

#ifndef SD_PIN_SCLK
#define SD_PIN_SCLK 5
#endif

#ifndef SD_PIN_MISO
#define SD_PIN_MISO 4
#endif

#ifndef SD_PIN_MOSI
#define SD_PIN_MOSI 6
#endif

#endif
