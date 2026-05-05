#pragma once

#include <ESP32Servo.h>
#include <etl/format_spec.h>
#include <etl/string.h>
#include <etl/to_string.h>

class ActuonixL12 {
  public: // MARK: Public
    static constexpr const char *TAG = "AL12";
    static constexpr std::pair<uint_fast16_t, uint_fast16_t> FEEDBACK_RANGE_mV =
        {2643, 407};
    static constexpr std::pair<uint_fast16_t, uint_fast16_t> PULSE_RANGE_us = {
        1000, 2000};

    /**
     * @see <docs/cpp/explicit_sonarLint.md>
     */
    explicit ActuonixL12(int pin, uint8_t feedbackPin, int min_us = 1000,
                         int max_us = 2000);
    ~ActuonixL12() = default;

    void begin();

    void writePosMicros(int us);

    uint_fast16_t readPos_mV();

    uint_fast16_t readPos_us();

    static constexpr uint_fast8_t LOG_STRING_SIZE = 3 + ((5 + 6) * 2) + 1;
    /**
     * @brief Get at string that describes the current state of the actuator
     * @returns the current state of the actuator as a string
     */
    etl::string<LOG_STRING_SIZE> getLogString();

  private: // MARK: Private Vars
    Servo servo_;
    int pin_;
    uint8_t feedbackPin_;
    int min_us_;
    int max_us_;
};
