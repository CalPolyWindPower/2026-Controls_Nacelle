#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>
#include <Wire.h>

#include "AS5600.h"
#include "NacelleConfig.hpp"

/**
 * @namespace Encoder
 * @brief Interface for initializing and reading RPM data from an AS5600 magnetic encoder.
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
// constexpr uint_fast32_t OPTIMAL_SAMPLE_TIME_mS = ENCODER::OPTIMAL_SAMPLE_TIME_mS;

// constexpr uint_fast16_t AVERAGING_PERIOD_MS = ENCODER::FILTER_HISTORY_SIZE * ENCODER::OPTIMAL_SAMPLE_TIME_mS;
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
 * @brief Computes the moving average of the encoder RPM.
 *
 * Updates the circular buffer with a new sample and returns
 * the current averaged RPM value.
 *
 * @param[out] rpmAvg Reference to store the computed RPM average.
 */
void getRpmMovingAverage(float& rpmAvg);

}  // namespace Encoder

#endif // ENCODER_HPP