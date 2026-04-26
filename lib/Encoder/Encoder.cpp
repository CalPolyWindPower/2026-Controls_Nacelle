#include "Encoder.hpp"

namespace Encoder {

AS5600 as5600;
float rpmSamples[DATASET_SIZE] = {0};
float runningRpmSum = 0;

/**
 * @brief Computes the moving average of the encoder RPM.
 *
 * Replaces the oldest sample in the buffer with a new measurement,
 * updates the running sum, and calculates the average RPM.
 *
 * @param[out] rpmAvg Reference to store the computed RPM average.
 */
void getRpmMovingAverage(std::atomic<int_fast16_t>& rpmAvg) {
  static uint_fast16_t index = 0;

  runningRpmSum -= rpmSamples[index];
  rpmSamples[index] = as5600.getAngularSpeed(AS5600_MODE_RPM);
  if(rpmSamples[index] == NAN) {
      ESP_LOGE(TAG, "AS5600 read failed");
  }
  runningRpmSum += rpmSamples[index];

  rpmAvg = (int_fast16_t)(runningRpmSum / DATASET_SIZE);

  if (index == (DATASET_SIZE - 1)) {
    index = 0;
  } else {
    index += 1;
  }
}

/**
 * @brief Checks the AS5600 error status.
 *
 * Outputs the error code via serial if the status is not OK.
 */
int errorChecking() {
  int e = as5600.lastError();
  if (e != AS5600_OK) {
    ESP_LOGE(TAG, "AS5600 error: %d", e);
  }

  return e;
}

/**
 * @brief Initializes the encoder and primes the RPM buffer.
 *
 * Starts I2C communication, configures the sensor, verifies connection,
 * and fills the sample buffer to ensure valid averaging during runtime.
 */
bool initialize() {
    Wire.begin(SDA_FIREBEETLE, SCL_FIREBEETLE);
    bool connected = as5600.begin();
    if(!connected) {
      ESP_LOGE(TAG, "AS5600 init failed");
      return false;
    }
    as5600.setDirection(AS5600_CLOCK_WISE);

    connected = as5600.isConnected();
    if(!connected) {
      ESP_LOGE(TAG, "AS5600 test failed");
      return false;
    }
    delay(1000); // todo maybe don't block in setup?

    for (int i = 0; i < DATASET_SIZE; i++) {
        rpmSamples[i] = as5600.getAngularSpeed(AS5600_MODE_RPM);
        if(rpmSamples[i] == NAN) {
            ESP_LOGE(TAG, "AS5600 read failed during init");
            return false;
        }
        runningRpmSum += rpmSamples[i];
        delay(SAMPLE_DELAY_MS); // todo maybe don't block in setup?
    }
    
    return true;
}

}  // namespace Encoder