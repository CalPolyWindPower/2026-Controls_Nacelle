/**
 * @file NacelleConfig.hpp
 */
#pragma once

static_assert(__cplusplus >= 202302L, "C++23 standard or later required.");

// Imports
#include <cstdint>
#define ESP32C5 1
#define BOARD ESP32C5
#if not(BOARD == ESP32C5)
#    warning "Not using production nacelle board!"
#endif

/**
 * @brief Debugging Setup
 * @see
 * https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/log.html
 */
#pragma region Debugging Setup

#define PROJECT_ID "WT26N" // CONFIG - Project ID to use with logger

#pragma endregion // Debugging Setup

// MARK: Boards

// Make sure the hardware pins are imported
#include <pins_arduino.h>

namespace FR_FIREBEETLE2_ESP32C6 {
    // Onboard LED
    constexpr uint_fast8_t LED_PIN = 15;

    // Actuator Pins
    constexpr uint_fast8_t ACTUATOR_PWM_PIN = 3;
    constexpr uint_fast8_t ACTUATOR_FB_PIN = 2;

    // I2C
    constexpr uint_fast8_t HPI2C_SDA_PIN = 9;
    constexpr uint_fast8_t HPI2C_SCL_PIN = 10;
    constexpr uint_fast8_t LPI2C_SDA_PIN = 6;
    constexpr uint_fast8_t LPI2C_SCL_PIN = 7;

    // SPI
    constexpr uint_fast8_t SPI_CIPO_PIN = 25;
    constexpr uint_fast8_t SPI_CLK_PIN = 23;
    constexpr uint_fast8_t SPI_COPI_PIN = 24;
    constexpr uint_fast8_t SPI_CS_PIN = 28;

    // UART
    constexpr uint_fast8_t UART_TX_PIN = TX;
    constexpr uint_fast8_t UART_RX_PIN = RX;
    constexpr uint_fast8_t LPUART_TX_PIN = LP_TX;
    constexpr uint_fast8_t LPUART_RX_PIN = LP_RX;
} // namespace FR_FIREBEETLE2_ESP32C6

// MARK: Constants
namespace CONSTS {
    constexpr uint32_t MILLIS_PER_SEC = 1000;
    constexpr uint32_t SECS_PER_MIN = 60;
} // namespace CONSTS

// Mark: Application
namespace LED {
    /**
     * @see
     * https://wiki.dfrobot.com/SKU_DFR1075_FireBeetle_2_Board_ESP32_C6#5.2%20LED%20Blinking
     */
    constexpr uint_fast8_t LED_PIN = FR_FIREBEETLE2_ESP32C6::LED_PIN;
    // constexpr uint32_t BLINK_OFF_SECS = 3;
    constexpr uint32_t BLINK_OFF_MILLIS = 3500;
    // constexpr uint32_t LED_BLINK_ON_SEC = 1;
    constexpr uint32_t BLINK_ON_MILLIS = CONSTS::MILLIS_PER_SEC / 3;

} // namespace LED

namespace RUN {
    constexpr uint32_t SLEEP_TIME_MINS = 10;
    constexpr uint32_t SLEEP_TIME_SECS = SLEEP_TIME_MINS * CONSTS::SECS_PER_MIN;
    constexpr uint32_t SLEEP_TIME_MILLIS =
        SLEEP_TIME_SECS * CONSTS::MILLIS_PER_SEC;
} // namespace RUN

namespace TELNET {
    constexpr uint_fast32_t SERIAL_SPEED = 115200;
    constexpr const char *WIFI_SSID = "CPWP-Nacelle";
    constexpr const char *WIFI_PASSWORD = "CowPolyWindPower";
    constexpr uint_fast8_t PORT = 23;
} // namespace TELNET