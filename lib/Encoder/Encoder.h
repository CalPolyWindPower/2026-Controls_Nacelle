#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include "AS5600.h"
#include <Wire.h"

/**
 * @namespace Encoder
 * @brief Interface for initializing and reading RPM data from an AS5600 magnetic encoder.
 */
namespace Encoder {

extern AS5600 as5600;

constexpr uint8_t SCL_FIREBEETLE = 10;
constexpr uint8_t SDA_FIREBEETLE = 9;

constexpr uint32_t AVERAGING_PERIOD_MS = 1000;
constexpr uint8_t DATASET_SIZE = 3;
constexpr uint32_t SAMPLE_DELAY_MS = AVERAGING_PERIOD_MS / DATASET_SIZE;

extern float rpmSamples[DATASET_SIZE];
extern float runningRpmSum;

/**
 * @brief Initializes the encoder and prepares the moving average buffer.
 *
 * Configures I2C communication, initializes the AS5600 sensor,
 * verifies connectivity, and preloads RPM samples.
 */
void initialize();

/**
 * @brief Checks for encoder communication or sensor errors.
 *
 * Prints the error code to the serial monitor if a fault is detected.
 */
void errorChecking();

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

#endif