#ifndef ENCODER_HPP
#define ENCODER_HPP

// System Libraries
#include <Arduino.h>
#include <Wire.h>
#include <atomic>

// Third Party Libraries
#include <etl/format_spec.h>
#include <etl/string.h>
#include <etl/to_string.h>

// Custom Includes
#include "AS5600.h"
#include "NacelleConfig.hpp"

/**
 * @namespace Encoder
 * @brief Interface for initializing and reading RPM data from an AS5600
 * magnetic encoder.
 */
namespace Encoder {

    static constexpr const char *TAG = "Enc";

    extern AS5600 as5600;

    /**
     * @brief I2C SCL pin
     * @details Wire.begin takes two ints for pins
     */
    constexpr int SCL_FIREBEETLE = 10;
    /**
     * @brief I2C SDA pin
     * @details Wire.begin takes two ints for pins
     */
    constexpr int SDA_FIREBEETLE = 9;

    // constexpr uint_fast16_t MAX_RPM = ENCODER::MAX_RPM;
    // constexpr uint_fast8_t MAX_RPS = ENCODER::MAX_RPS;
    // constexpr uint_fast16_t MIN_T_mS_PER_REV = ENCODER::MIN_T_mS_PER_REV;
    // constexpr uint_fast32_t OPTIMAL_SAMPLE_TIME_mS =
    // ENCODER::OPTIMAL_SAMPLE_TIME_mS;

    // constexpr uint_fast16_t AVERAGING_PERIOD_MS =
    // ENCODER::FILTER_HISTORY_SIZE * ENCODER::OPTIMAL_SAMPLE_TIME_mS;
    constexpr uint_fast8_t DATASET_SIZE = ENCODER::FILTER_HISTORY_SIZE;
    constexpr uint32_t SAMPLE_DELAY_MS = ENCODER::OPTIMAL_SAMPLE_TIME_mS;

    extern float rpmSamples[DATASET_SIZE];
    extern float runningRpmSum;

    /**
     * @brief Initializes the encoder and prepares the moving average buffer.
     *
     * @returns true if initialization is successful, false otherwise.
     *
     * Configures I2C communication, initializes the AS5600 sensor,
     * verifies connectivity, and preloads RPM samples.
     */
    bool initialize();

    /**
     * @brief Checks for encoder communication or sensor errors.
     *
     * @brief Returns the error code
     *
     * Prints the error code to the serial monitor if a fault is detected.
     */
    int errorChecking();

    /**
     * @brief Check for C++17 support, which allows us to verify if std::atomic
     * is a acceptable (lock free) solution for shared variables
     * @see https://stackoverflow.com/a/49915536
     */
    static_assert(
        (__cplusplus >= 201703L),
        "C++17 or higher is required for std::atomic is_always_lock_free");

    /**
     * @brief Do some basic checks regarding std::atomic and data types
     * From C++14.2.0 atomic.h:
     * Check Lock-free property.
     *
     * 0 indicates that the types are never lock-free.
     * 1 indicates that the types are sometimes lock-free.
     * 2 indicates that the types are always lock-free.
     */
    static_assert(sizeof(int) == sizeof(int_fast16_t),
                  "Atomic Lock-free check issue");
// Can't use floats atomically
// #if (ATOMIC_FLOAT_LOCK_FREE == 0)
// #    error "Atomic operations on float are not lock-free on this platform."
// #elif (ATOMIC_FLOAT_LOCK_FREE == 1)
// #    warning \
//         "Atomic operations on float are only sometimes lock-free on this
//         platform."
// #endif
#if (ATOMIC_INT_LOCK_FREE == 0)
#    error "Atomic operations on int are not lock-free on this platform."
#elif (ATOMIC_INT_LOCK_FREE == 1)
#    warning                                                                   \
        "Atomic operations on int are only sometimes lock-free on this platform."
#endif

    /**
     * @brief Check if std::atomic<int_fast16_t> is an acceptable (lock free)
     * solution for shared variables
     * @see https://www.reddit.com/r/embedded/comments/zn23of/comment/j0fav6o/
     * @see
     * https://stackoverflow.com/questions/63471387/should-volatile-still-be-used-for-sharing-data-with-isrs-in-modern-c
     * @see https://en.cppreference.com/w/c/language/atomic.html
     * @see https://en.cppreference.com/w/cpp/atomic/atomic.html
     * @see https://stackoverflow.com/a/16783513
     */
    static_assert(std::atomic<int_fast16_t>::is_always_lock_free,
                  "Atomic operations on int_fast16_t are not lock-free on "
                  "this platform.");

    /**
     * @brief Computes the moving average of the encoder RPM.
     *
     * Updates the circular buffer with a new sample and returns
     * the current averaged RPM value.
     *
     * @param[out] rpmAvg Reference to store the computed RPM average.
     */
    void getRpmMovingAverage(std::atomic<int_fast16_t> &rpmAvg);

    static constexpr uint_fast8_t LOG_STRING_SIZE =
        3 + 6 + 2 + 6 + 2 + 7 + 2 + 5 + 1 + 3 + ((6 + 1) * 3) + 1 + 6 + 4 + 4;
    /**
     * @brief Get at string that describes the current state of the PID instance
     * @returns the current state of the PID instance as a string
     */
    etl::string<LOG_STRING_SIZE> getLogString();

} // namespace Encoder

#endif // ENCODER_HPP