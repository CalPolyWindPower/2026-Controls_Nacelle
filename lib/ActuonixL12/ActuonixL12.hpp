#pragma once

#include <ESP32Servo.h>
#include <etl/format_spec.h>
#include <etl/string.h>
#include <etl/to_string.h>

class ActuonixL12 {
  public: // MARK: Public
    static constexpr const char *TAG = "AL12";

    /**
     * @see <docs/cpp/explicit_sonarLint.md>
     */
    explicit ActuonixL12(int pin, int min_us = 1000, int max_us = 2000);
    ~ActuonixL12() = default;

    void begin();

    void writePosMicros(int us);

    static constexpr uint_fast8_t LOG_STRING_SIZE = 3 + 6 + 1;
    /**
     * @brief Get at string that describes the current state of the actuator
     * @returns the current state of the actuator as a string
     */
    etl::string<LOG_STRING_SIZE> getLogString();

  private: // MARK: Private Vars
    Servo servo_;
    int pin_;
    int min_us_;
    int max_us_;
};
