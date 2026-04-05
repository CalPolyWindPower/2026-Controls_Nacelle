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
    /**
     * @brief Actuator PWM control pin
     * @details Control with pulse width modulation (PWM): Hobby RC style
     * (1,000-2,00 microseconds pulses every 20,0000 microsecond period) or by
     * duty cycle at 1 kHz)
     */
    constexpr uint_fast8_t ACTUATOR_PWM_PIN = 3;
    constexpr uint_fast8_t ACTUATOR_FB_PIN = 2;

    // I2C
    constexpr uint_fast8_t HPI2C_SDA_PIN = 9;  // High Power I2C Serial Data
    constexpr uint_fast8_t HPI2C_SCL_PIN = 10; // High Power I2C Serial Clock
    constexpr uint_fast8_t LPI2C_SDA_PIN = 6;  // Low Power I2C Serial Data
    constexpr uint_fast8_t LPI2C_SCL_PIN = 7;  // Low Power I2C Serial Clock

    // SPI
    /**
     * @brief SPI Controller In Peripheral Out (MISO)
     */
    constexpr uint_fast8_t SPI_CIPO_PIN = 25;
    constexpr uint_fast8_t SPI_CLK_PIN = 23; // SPI Clock
    /**
     * @brief SPI Controller Out Peripheral In (MOSI)
     */
    constexpr uint_fast8_t SPI_COPI_PIN = 24;
    constexpr uint_fast8_t SPI_CS_PIN = 28; // SPI Chip Select (SS)

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

namespace SENSORS {
    constexpr uint16_t ENCODER_MIN_MEAS_TIME_DELTA_MS = 0;    // CONFIG
    constexpr uint16_t ENCODER_MAX_MEAS_TIME_DELTA_MS = 1000; // CONFIG
    constexpr uint16_t ENCODER_FILTER_HISTORY_SIZE = 10;      // CONFIG

    constexpr uint16_t TARGET_RPM = 2750;              // CONFIG
    constexpr uint16_t CURTAIL_HYSTERESIS_UP = 0;      // CONFIG
    constexpr uint16_t CURTAIL_HYSTERESIS_DOWN = 1000; // CONFIG
    constexpr uint16_t FINE_CONTROL_MIN_RPM = 200;     // CONFIG
    constexpr uint16_t MAX_RATED_RPM = 6000;           // CONFIG
} // namespace SENSORS

namespace PITCHING {
    // polynomial fits based on updated shaft (5/6/25) to modify CMM calibration
    // this is the updated version after the actuator mounting hole was shifted
    // forward 2mm.
    constexpr float SERVO_A0 = 562.9471905;
    constexpr float SERVO_A1 = -0.634880198;
    constexpr float SERVO_A2 = 0.000178592;

    // Mechanism constants
    constexpr uint16_t PITCH_MIN_ANGLE_DEG = 0;
    constexpr uint16_t PITCH_CUTIN_ANGLE_DEG = 9;
    constexpr uint16_t PITCH_MAX_ANGLE_DEG = 85;
    constexpr uint16_t ARM_CHORD_DEG_CCW =
        100; // angle between arm and tip chord (+ccw looking from tip)
    constexpr uint16_t PITCH_ANGLE_OFFSET = ARM_CHORD_DEG_CCW - 90;

    constexpr uint16_t SERVO_MIN_uS_2026 = 1000; // todo
    constexpr uint16_t SERVO_MAX_uS_2026 = 2000; // todo
    constexpr uint16_t SERVO_MIN_uS_2025 = 1000;
    constexpr uint16_t SERVO_MAX_uS_2025 = 2000;
    constexpr uint16_t SERVO_MIN_uS_2024 = 1450;
    constexpr uint16_t SERVO_MAX_uS_2024 = 1870;

    // Pitching config
    constexpr uint8_t BLADE_PITCH_STARTUP_DEG =
        PITCH_CUTIN_ANGLE_DEG; // CONFIG //todo
    constexpr uint16_t POS_STARTUP_uS =
        SERVO_MAX_uS_2025; // CONFIG //todo

    constexpr uint8_t BLADE_PITCH_STOP_DEG =
        PITCH_MAX_ANGLE_DEG; // Feather - CONFIG //todo
    constexpr uint16_t POS_STOP_uS =
        SERVO_MIN_uS_2025; // Feather -CONFIG //todo

    // todo: Ftarget rpm var

    /* PID Config */
    constexpr float PITCH_Kp = 0.005f;
    constexpr float PITCH_Ki = 0.001f;
    constexpr float PITCH_Kd = 0.0f;
} // namespace PITCHING

namespace RUN {
    // Task Execution Intervals
    enum TASK_INTERVALS : uint32_t {
        TI_FSM_mS = 100,        // CONFIG - 100 ms (10 Hz)
        TI_POLL_SENSORS_mS = 2, // CONFIG - 2 ms (500 Hz)
        TI_PITCH_mS = 10,       // CONFIG - 10 ms (100 Hz)
        TI_RECV_ms = 100,       // CONFIG - 100 ms (10 Hz)
        TI_SEND_ms = 10,        // CONFIG - 10 ms (100 Hz)
        TI_CFG_ms = 1000,       // CONFIG - 1000 ms (1 Hz)
        TI_TELNET_ms = 500,     // CONFIG - 500 ms (2 Hz)
        TI_OTA_ms = 1000,       // CONFIG - 1000 ms (1 Hz)
        TI_LOG_DATA_ms = 4000   // CONFIG - 4000 ms (0.25 Hz)
    };

    // todo: What was this for?
    // constexpr uint32_t SLEEP_TIME_MINS = 10;
    // constexpr uint32_t SLEEP_TIME_SECS = SLEEP_TIME_MINS *
    // CONSTS::SECS_PER_MIN; constexpr uint32_t SLEEP_TIME_MILLIS =
    //     SLEEP_TIME_SECS * CONSTS::MILLIS_PER_SEC;
} // namespace RUN

namespace TELNET {
    constexpr uint_fast32_t SERIAL_SPEED = 115200;
    constexpr const char *WIFI_SSID = "CPWP-Nacelle";
    constexpr const char *WIFI_PASSWORD = "CowPolyWindPower";
    constexpr uint_fast8_t PORT = 23;
} // namespace TELNET