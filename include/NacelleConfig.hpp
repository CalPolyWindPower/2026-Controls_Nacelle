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

#define PROJECT_ID "WT26N" // CONFIG - Project ID to use with logger // NOSONAR

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
    constexpr uint_fast32_t MILLIS_PER_SEC = 1000;
    constexpr uint_fast32_t SECS_PER_MIN = 60;
    constexpr uint_fast16_t uVOLTS_PER_VOLT = 1'000'000;
} // namespace CONSTS

// Mark: Application
namespace LED {
    /**
     * @see
     * https://wiki.dfrobot.com/SKU_DFR1075_FireBeetle_2_Board_ESP32_C6#5.2%20LED%20Blinking
     */
    constexpr uint_fast8_t PIN = FR_FIREBEETLE2_ESP32C6::LED_PIN;
    // constexpr uint_fast32_t BLINK_OFF_SECS = 3;
    constexpr uint_fast32_t BLINK_OFF_MILLIS = 3500;
    // constexpr uint_fast32_t LED_BLINK_ON_SEC = 1;
    constexpr uint_fast32_t BLINK_ON_MILLIS = CONSTS::MILLIS_PER_SEC / 3;
} // namespace LED

namespace ENCODER {
    constexpr uint_fast16_t START_RUN_2_RPM = 500; // CONFIG
    constexpr uint_fast16_t TARGET_RPM = 2200;     // CONFIG
    constexpr uint_fast16_t MAX_RPM = 3000;        // CONFIG
    constexpr uint_fast8_t MAX_RPS = MAX_RPM / CONSTS::SECS_PER_MIN;
    constexpr uint_fast16_t MIN_T_mS_PER_REV = 1000 / MAX_RPS;
    constexpr uint_fast32_t OPTIMAL_SAMPLE_TIME_mS = MIN_T_mS_PER_REV / 4;
    // static_assert(SAMPLE_DELAY_MS == OPTIMAL_SAMPLE_TIME_mS,
    //               "Suboptimal encoder sample time.");

    constexpr uint_fast16_t MEAS_TIME_DELTA_MIN_MS = 0; // CONFIG
    /**
     * @brief [CONFIG] Sets delay between samples so that collecting
     * DATASET_SIZE samples spans one full averaging period
     */
    constexpr uint_fast16_t MEAS_TIME_DELTA_MS = OPTIMAL_SAMPLE_TIME_mS;
    /**
     * @brief [CONFIG] size of the rpmSamples array and # of times encoder
     * samples per second
     * @details was 10 last year
     */
    constexpr uint_fast16_t FILTER_HISTORY_SIZE = 8; // CONFIG
    /**
     * @brief [CONFIG] Time window of moving average in milliseconds
     * @details 2 ms last year. Pitch was every 10 ms
     */
    // constexpr uint_fast16_t AVERAGING_PERIOD_MS =
    //     FILTER_HISTORY_SIZE * MEAS_TIME_DELTA_MS; // CONFIG

    // todo check avg. function - can a neg. ruin it? - not problem?

    constexpr uint_fast16_t CURTAIL_HYSTERESIS_UP = 0;      // CONFIG
    constexpr uint_fast16_t CURTAIL_HYSTERESIS_DOWN = 1000; // CONFIG
    constexpr uint_fast16_t FINE_CONTROL_MIN_RPM = 200;     // CONFIG
    constexpr uint_fast16_t MAX_RATED_RPM = 3500;           // CONFIG
    constexpr uint_fast16_t MOTOR_kV_RPMPV = 107;
    constexpr uint_fast16_t MOTOR_IkV_RPSPuV =
        CONSTS::uVOLTS_PER_VOLT * CONSTS::SECS_PER_MIN / MOTOR_kV_RPMPV;
} // namespace ENCODER

namespace PITCHING {
    // polynomial fits based on updated shaft (5/6/25) to modify CMM calibration
    // this is the updated version after the actuator mounting hole was shifted
    // forward 2mm.
    constexpr float SERVO_A0 = 562.9471905;
    constexpr float SERVO_A1 = -0.634880198;
    constexpr float SERVO_A2 = 0.000178592;

    // Mechanism constants
    // constexpr uint_fast16_t PITCH_MIN_ANGLE_DEG = 0;
    // constexpr uint_fast16_t PITCH_CUTIN_ANGLE_DEG = 9;
    // constexpr uint_fast16_t PITCH_MAX_ANGLE_DEG = 85;
    // constexpr uint_fast16_t ARM_CHORD_DEG_CCW =
    //     100; // angle between arm and tip chord (+ccw looking from tip)
    // constexpr uint_fast16_t PITCH_ANGLE_OFFSET = ARM_CHORD_DEG_CCW - 90;

    constexpr uint_fast16_t SERVO_MIN_uS_2026 = 1230;
    constexpr uint_fast16_t SERVO_MAX_uS_2026 = 1900;
    constexpr uint_fast16_t SERVO_MIN_uS_2025 = 1000;
    constexpr uint_fast16_t SERVO_MAX_uS_2025 = 2000;
    constexpr uint_fast16_t SERVO_MIN_uS_2024 = 1450;
    constexpr uint_fast16_t SERVO_MAX_uS_2024 = 1870;

    // Pitching config
    // constexpr uint_fast8_t BLADE_PITCH_STARTUP_DEG =
    //     PITCH_CUTIN_ANGLE_DEG;                                  // CONFIG
    //     //todo
    constexpr uint_fast16_t POS_STARTUP_uS = 1540; // CONFIG //todo
    constexpr uint_fast16_t POS_RUN_uS = 1640;     // CONFIG //todo

    // constexpr uint_fast8_t BLADE_PITCH_STOP_DEG =
    //     PITCH_MAX_ANGLE_DEG; // Feather - CONFIG //todo
    constexpr uint_fast16_t POS_STOP_uS = SERVO_MIN_uS_2026; // Feather - CONFIG

    // todo: Ftarget rpm var

    /* PID Config */
    constexpr float PITCH_Kp = 0.127f; // CONFIG - 0.005f last year
    constexpr float PITCH_Ki = 0.000f; // CONFIG - 0.001f last year
    constexpr float PITCH_Kd = 0.0f;
} // namespace PITCHING

namespace RUN {
    // Task Execution Intervals
    enum TASK_INTERVALS : uint_fast32_t {
        TI_FSM_mS = 100,        // CONFIG - 100 ms (10 Hz)
        TI_POLL_SENSORS_mS = 2, // CONFIG - 2 ms (500 Hz) // todo - change?
        TI_PITCH_mS = 10,       // CONFIG - 10 ms (100 Hz)
        TI_RECV_ms = 50, // CONFIG - 100 ms (10 Hz)
        TI_SEND_ms =
            ENCODER::MEAS_TIME_DELTA_MS, // CONFIG - 5 - 10 ms (200 - 100 Hz)
        TI_CFG_ms = 1000,                // CONFIG - 1000 ms (1 Hz)
        TI_TELNET_ms = 500,              // CONFIG - 500 ms (2 Hz)
        TI_OTA_ms = 1000,                // CONFIG - 1000 ms (1 Hz)
        TI_LOG_DATA_ms = 4000            // CONFIG - 4000 ms (0.25 Hz)
    };

    // todo: What was this for?
    // constexpr uint_fast32_t SLEEP_TIME_MINS = 10;
    // constexpr uint_fast32_t SLEEP_TIME_SECS = SLEEP_TIME_MINS *
    // CONSTS::SECS_PER_MIN; constexpr uint_fast32_t SLEEP_TIME_MILLIS =
    //     SLEEP_TIME_SECS * CONSTS::MILLIS_PER_SEC;
} // namespace RUN

namespace TELNET {
    constexpr uint_fast32_t SERIAL_SPEED = 115200;
    constexpr const char *WIFI_SSID = "CPWP-Nacelle";
    constexpr const char *WIFI_PASSWORD = "CowPolyWindPower";
    constexpr uint_fast8_t PORT = 23;
} // namespace TELNET