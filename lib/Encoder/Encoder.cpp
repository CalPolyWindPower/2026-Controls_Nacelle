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
  /**
   * @details From GitHub Copilot, GPT-5.4 mini: 
   * @see
   * https://stackoverflow.com/questions/570669/checking-if-a-double-or-float-is-nan-in-c
   */
  if(std::isnan(rpmSamples[index])) {
      ESP_LOGE(TAG, "AS5600 read failed");
      // TODO: Handle better?
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

/**
 * @see https://stackoverflow.com/questions/3350385/how-to-return-an-object-in-c
 */
etl::string<Encoder::LOG_STRING_SIZE> getLogString() {
    etl::string<LOG_STRING_SIZE> logString(TAG); // 3 chars
    logString.append(": DS: ");                 // 6 chars

    etl::format_spec decFormatA;
    decFormatA.width(2).fill('0');                                // [2 chars]
    etl::to_string(DATASET_SIZE, logString, decFormatA, true);    // 2 chars
    logString.append(", SD: ");                                   // 6 chars
    etl::to_string(SAMPLE_DELAY_MS, logString, decFormatA, true); // 2 chars
    logString.append(", A: 0x");                                  // 7 chars

    etl::format_spec hexFormatA;
    hexFormatA.hex().width(2).fill('0');                              // [2 chars]
    etl::to_string(as5600.getAddress(), logString, hexFormatA, true); // 2 chars
    logString.append(", D: ");                                        // 5 chars          

    etl::format_spec boolFormatA;
    etl::to_string(as5600.getDirection(), logString, boolFormatA, true); // 1 char
    logString.append(", AGC: ");

    etl::format_spec decFormatB;
    decFormatB.width(3).fill('0');                             // [3 chars]
    etl::to_string(DATASET_SIZE, logString, decFormatB, true); // 3 chars
    logString.append(", MD: ");                                // 6 chars
    
    boolFormatA.binary().width(1).fill('0');                               // [1 char]
    etl::to_string(as5600.detectMagnet(), logString, decFormatB, true);    // 1 chars
    logString.append(", MTS: ");                                           // 6 chars
    etl::to_string(as5600.magnetTooStrong(), logString, decFormatB, true); // 1 chars
    logString.append(", MTW: ");                                           // 6 chars
    etl::to_string(as5600.magnetTooWeak(), logString, decFormatB, true);   // 1 chars
    logString.append(", lE: ");                                            // 6 chars

    etl::format_spec decFormatC;
    decFormatC.width(4).fill('0');                                   // [4 chars]
    etl::to_string(as5600.lastError(), logString, decFormatC, true); // 4 chars
    

    return logString;
};

}  // namespace Encoder