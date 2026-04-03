#include <Arduino.h>
#include "AS5600.h"
#include <Wire.h>



namespace Encoder {
  AS5600 as5600;

  constexpr uint8_t SCL_FIREBEETLE = 10; // SCL pin on the Firebeetle
  constexpr uint8_t SDA_FIREBEETLE = 9;  // SDA pin on the Firebeetle

  constexpr uint32_t AVERAGING_PERIOD_MS = 1000; // time window of moving average, in milliseconds
  constexpr uint8_t DATASET_SIZE = 3;            // size of the rpmSamples array and # of times encoder samples per second

  // Sets delay between samples so that collecting DATASET_SIZE samples spans one full averaging period
  constexpr uint32_t SAMPLE_DELAY_MS = AVERAGING_PERIOD_MS / DATASET_SIZE; 

  float rpmSamples[DATASET_SIZE]; // Circular buffer of recent RPM samples
  float runningRpmSum = 0;        // Sum of the RPM samples currently stored in rpmSamples

  void getRpmMovingAverage(float& rpmAvg);
  void errorChecking();
  void initialize();
}



void setup() {
  Serial.begin(115200);
  Encoder::initialize();
}



void loop() {
  Encoder::errorChecking();

  float rpmAverage;
  Encoder::getRpmMovingAverage(rpmAverage);

  Serial.print("\tω = ");
  Serial.println(rpmAverage, 3); // Print average RPM with 3 decimal places

  delay(Encoder::SAMPLE_DELAY_MS);
}

// Updates the moving average of the RPM over the averaging period
void Encoder::getRpmMovingAverage(float& rpmAvg) {
  static uint16_t index = 0; // Index of the oldest RPM sample

  Encoder::runningRpmSum -= Encoder::rpmSamples[index];                           // Subtracts the oldest rpm sample from the running sum
  Encoder::rpmSamples[index] = Encoder::as5600.getAngularSpeed(AS5600_MODE_RPM);  // Replaces oldest rpm sample with newest
  Encoder::runningRpmSum += Encoder::rpmSamples[index];                           // Adds the newest rpm sample to the running sum

  rpmAvg = Encoder::runningRpmSum / Encoder::DATASET_SIZE;  // RPM average: Quotient of the running sum of the RPM samples and the size of the dataset

  // loops index back to the start 
  if (index == (Encoder::DATASET_SIZE - 1)) {
    index = 0;
  }
  else {
    index += 1;
  }
}

// Definitely room for a more in depth system
void Encoder::errorChecking() {
  int e = Encoder::as5600.lastError();
  if (e != AS5600_OK){
    Serial.println(e);
  }
}

// runs the setup for the encoder
void Encoder::initialize() {
  Wire.begin(Encoder::SDA_FIREBEETLE, Encoder::SCL_FIREBEETLE);
  Encoder::as5600.begin(); 
  Encoder::as5600.setDirection(AS5600_CLOCK_WISE);    // sets encoder's assumed direction of rotation to clockwise 
  int connectionTest = Encoder::as5600.isConnected(); // checks if the microcontroller has successfully established a connection with the encoder
  Serial.print("Connect: ");
  Serial.println(connectionTest);
  delay(1000);

  // load RPM samples into array to prevent error during main loop
  for (int i = 0; i < Encoder::DATASET_SIZE; i++) {
    Encoder::rpmSamples[i] = Encoder::as5600.getAngularSpeed(AS5600_MODE_RPM);
    Encoder::runningRpmSum += Encoder::rpmSamples[i];
    delay(Encoder::SAMPLE_DELAY_MS);
  }
}
