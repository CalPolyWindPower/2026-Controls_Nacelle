/**
 * @file nacelleDemo.ino
 * @brief Manually control the actuator
 * @version 1.1.0
 * @since Winter 2026
 * @author Noah (@BobSaidHi <https://github.com/bobsaidhi>) for
 * @CalPolyWindPower <https://github.com/calpolywindpower>
 * @author Inspired by Trevor (@rover-t <https://github.com/rover-t>) at
 * @CalPolyWindPower <https://github.com/calpolywindpower>
 *
 * The DFRobot FireBeetle 2 ESP32-C5 has one USB C port.  The baud rate is set
 * to to 115,200 below and uses the internal USB peripheral, not a separate
 * chip. Some compatible serial monitors include the Arduino IDE
 * <https://www.arduino.cc/en/software/>, VSCode "Serial" extension from
 * Microsoft
 * <https://marketplace.visualstudio.com/items?itemName=ms-vscode.vscode-serial-monitor>,or
 * PuTTY.
 *
 * To re-flash:
 * 1. Install Arduino IDE (I recommend >v2.0) from
 * https://www.arduino.cc/en/software/ if you don't have it.
 * 2. Go to the boards manager in the left sidebar and install
 * "esp32" by Espressif Systems.
 *   A. If it's missing, go to the following board URL under
 *      File > Preferences > Settings >
 *      Additional Board Manager URLS:
 *      `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
 *      (See also:
 *       https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html)
 * 3. Load the attached sketch or the latest version from GitHub:
 *    https://github.com/CalPolyWindPower/2026-Controls_Nacelle/blob/main/demos/ActuatorDemo/ActuatorDemo.ino
 * 4. Set the board to "ESP32C5 Dev Module".  There may be multiple
 *    COM / serial ports to choose from.  If one is labeled "ESP32
 *    Family Device", use that and switch it to "ESP32C5 Dev Module".
 * 5. Under Tools, set "USB CDC On Boot" to "Enabled"
 * 6. [Recommended] Under Tools, set "Core Debug Level" to "Info"
 * 7. Select Upload
 * 8. Open a serial terminal
 *   A. In Arduino IDE, got Tools > Serial Monitor and switch to 115200 baud
 *
 * @author Noah (@BobSaidHi <https://github.com/bobsaidhi>) for Cal Poly Wind
 * Power (@calpolywindpower <https://github.com/calpolywindpower>)
 * @see Inspired by
 * https://github.com/rover-t/CPWP2024-Controls/blob/main/ControlCode/OpenLoopController/OpenLoopController.ino
 * @see Inspired by
 * https://github.com/rover-t/CPWP2024-Controls/blob/main/ControlCode/OpenLoopController_alt/OpenLoopController_alt.ino
 */

// MARK: Imports
#include "AS5600.h"
#include <Arduino.h>
#include <Wire.h>
#include <cstdint> // Fixed size integers
// #include <pins_arduino.h>
/**
 * @details Only use ESP32 Servo on the ESP32, and use our custom one for better
 * ESP32 C5 support.
 * @see https://github.com/madhephaestus/ESP32Servo/blob/master/src/ESP32Servo.h
 * @see https://docs.arduino.cc/libraries/servo/
 */
#if defined(ESP32)
// #define ENFORCE_PINS 0
// #include <ESP32Servo.h>
#    include "./src/ESP32Servo/src/ESP32PWM.h"
#    include "./src/ESP32Servo/src/ESP32Servo.h"
#else
#    warning "Not using production board!"
#    include <Servo.h>
#endif

// MARK: Globals

/**
 * @brief Configuration for serial (UART) communication
 * @details A namespace can be used to organize related constants, variables,
 * and objects together.
 * @see https://www.geeksforgeeks.org/cpp/namespace-in-c/
 * @details `constexpr` is similar to `#define` macros but type checked.  Macros
 * and  `constexpr` variables are guaranteed and to be evaluated at compile
 * time, and may even be embedded in the immediate field of an instruction,
 * which means they do not take up additional storage space, memory, or
 * registers at turn time.
 * @see https://en.cppreference.com/w/cpp/language/constexpr.html
 * @see https://www.sciencedirect.com/topics/computer-science/immediate-operand
 */
namespace SERIAL_CONFIG {
    constexpr long BAUD = 115200; // BAUD rate (bits per second)
} // namespace SERIAL_CONFIG

/**
 * @brief Configuration for the actuator and related pins
 * @details The standard integer types, such as int, only specify a minimum
 * size.  FEEDBACK_PIN is of the fixed width unsigned integer type `uint8_t`,
 * which is always 8 bits.  This was chosen because the `pinmode()` function
 * takes an `uint8_t` as an argument.
 */
namespace Actuator {
    Servo device = Servo();
    constexpr int MIN_POS_us =
        1230; // Minimum position in microseconds // max in break
    constexpr int MAX_POS_us =
        1900; // Maximum position in microseconds // max out, stall at 1645,
              // even at higher wind speeds
    constexpr int DEFAULT_us =
        1550; // Default position in microseconds // cut in-ish
    constexpr int CONTROL_PIN = 3; // PWM pin
    constexpr uint8_t FEEDBACK_PIN =
        2; // Analog feedback pin
           // cut in 1480 us @ R=0b1 / 8.75 Ohms, 1540 @ 24 Ohms
} // namespace Actuator

/**
 * @brief Configuration for the LED and related pins
 * @details As we have more program storage space, and hopefully more RAM than
 * we need, the slight performance boost from the fast integer types, such ast
 * uint_fast32_t, are likely worth it for us.  These types specify a minium
 * size, but allow the compiler (actually the system/ standard libraries) to
 * substitute a type that a particular chip (micro-architecture) is more
 * optimized for.
 */
namespace LED {
    constexpr uint8_t PIN = 15; // LED pin
    constexpr uint_fast32_t TIME_ON_MS =
        1000; // Time LED stays on in milliseconds
    constexpr uint_fast32_t TIME_OFF_MS =
        3000; // Time LED stays off in milliseconds
} // namespace LED

namespace Encoder {
    AS5600 as5600;

    constexpr uint8_t SCL_FIREBEETLE = 10; // SCL pin for the Firebeetle
    constexpr uint8_t SDA_FIREBEETLE = 9;  // SDA pin for the Firebeetle

    constexpr uint32_t AVERAGING_PERIOD_MS =
        40; // time window of moving average, in milliseconds
    constexpr uint8_t DATASET_SIZE = 8; // size of the rpmSamples array and # of
                                        // times encoder samples per second

    // Sets delay between samples so that collecting DATASET_SIZE samples spans
    // one full averaging period
    constexpr uint32_t SAMPLE_DELAY_MS = AVERAGING_PERIOD_MS / DATASET_SIZE;
    // todo check avg. function - can a neg. ruin it? - not problem?

    // TODO: Max RPM
    constexpr uint_fast16_t MAX_RPM = 3000;
    constexpr uint_fast8_t SECS_PER_MIN = 60;
    constexpr uint_fast8_t MAX_RPS = MAX_RPM / SECS_PER_MIN;
    constexpr uint_fast16_t MIN_T_mS_PER_REV = 1000 / MAX_RPS;
    constexpr uint_fast32_t OPTIMAL_SAMPLE_TIME_mS = MIN_T_mS_PER_REV / 4;
    static_assert(SAMPLE_DELAY_MS == OPTIMAL_SAMPLE_TIME_mS, "Suboptimal encoder sample time.");
    // constexpr uint_fast16_t ENCODER_TICKS_PER_REV = 4092;
    // constexpr uint_fast32_t MAX_ENCODER_TICS_PER_SEC = MAX_RPS *
    // ENCODER_TICKS_PER_REV;

    float rpmSamples[DATASET_SIZE]; // Circular buffer of recent RPM samples
    float runningRpmSum =
        0; // Sum of the RPM samples currently stored in rpmSamples

  void getRpmMovingAverage(float& rpmAvg);
  void errorChecking();
  void initialize();
} // namespace encoder

/**
 * @details put your setup code here, to run once:
 */
void setup() {
    Serial.begin(SERIAL_CONFIG::BAUD); // Start serial

// The DFRobot FireBeetle 2 ESP32-C5 has one user controllable LED built in
#if defined(ESP32)
    /**
     * @see
     * https://docs.arduino.cc/language-reference/en/functions/digital-io/pinMode/
     */
    pinMode(LED::PIN, OUTPUT);    // Setup LED
    digitalWrite(LED::PIN, HIGH); // Toggle LED

    // Debug actuator pin
    // pinMode(Actuator::CONTROL_PIN, OUTPUT);     // Setup LED
    // digitalWrite(Actuator::CONTROL_PIN, HIGH);  // Toggle LED
    // while(1) {}
#endif

    // Servo device = Servo();

    /**
     * @details The Servo object, device, is in the Actuator namespace
     */
    Actuator::device.attach(Actuator::CONTROL_PIN, Actuator::MIN_POS_us,
                            Actuator::MAX_POS_us); // Attach actuator
    Actuator::device.writeMicroseconds(Actuator::DEFAULT_us);

    Serial.println(
        "Initalized 2026 Actuator and Encoder Demo v2026-4-2 / v1.0.0");
    digitalWrite(LED::PIN, LOW); // Toggle LED

    Encoder::initialize();
    digitalWrite(LED::PIN, LOW); // Toggle LED

    Serial.print("Input servo position in Microseconds: ");
}

/**
 * @details put your main code here, to run repeatedly:
 */
void loop() {
    // Check if Serial data is available
    if (Serial.available() > 0) {
        int position = Serial.parseInt();

        // It would appear parseInt returns 0 on failure
        if (position != 0) {
            // Check and write position
            if (position < Actuator::MIN_POS_us) {
                Serial.print("\nposition too small: ");
            } else if ((position > Actuator::MAX_POS_us)) {
                Serial.print("\nposition too large: ");
            } else {
                Serial.print("\nNew Position: ");
                Actuator::device.writeMicroseconds(position);
            }
            Serial.println(position);

            Serial.print("Input servo position in Microseconds: ");
        }
    }

#if defined(ESP32)
    // Toggle LED because why not, but only on the ESP32
    static int lastLEDTime_ms = 0; // Track time toggled
    static bool LEDOn = false;     // Track LED state

    // Check LED state and update
    if (!LEDOn && millis() - lastLEDTime_ms > LED::TIME_OFF_MS) {
        digitalWrite(LED::PIN, HIGH); // Toggle LED
        lastLEDTime_ms = millis();    // Save time
        LEDOn = true;
    } else if (LEDOn && millis() - lastLEDTime_ms > LED::TIME_ON_MS) {
        digitalWrite(LED::PIN, LOW); // Toggle LED
        lastLEDTime_ms = millis();   // Save time
        LEDOn = false;
    }
#endif

  Encoder::errorChecking(); // checks for basic I2C errors

  float rpmAverage;
  Encoder::getRpmMovingAverage(rpmAverage);

  Serial.print("\tω = ");
  Serial.println(rpmAverage, 3); // Print average RPM with 3 decimal places

  delay(Encoder::SAMPLE_DELAY_MS);

}

// Updates the moving average of the RPM over the averaging period
void Encoder::getRpmMovingAverage(float &rpmAvg) {
    static uint16_t index = 0; // Index of the oldest RPM sample

    Encoder::runningRpmSum -=
        Encoder::rpmSamples[index]; // Subtracts the oldest rpm sample from the
                                    // running sum
    Encoder::rpmSamples[index] = Encoder::as5600.getAngularSpeed(
        AS5600_MODE_RPM); // Replaces oldest rpm sample with newest
    Encoder::runningRpmSum +=
        Encoder::rpmSamples[index]; // Adds the newest rpm sample to the running
                                    // sum

    rpmAvg =
        Encoder::runningRpmSum /
        Encoder::DATASET_SIZE; // RPM average: Quotient of the running sum of
                               // the RPM samples and the size of the dataset

    // loops index back to the start
    if (index == (Encoder::DATASET_SIZE - 1)) {
        index = 0;
    } else {
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
        Encoder::rpmSamples[i] =
            Encoder::as5600.getAngularSpeed(AS5600_MODE_RPM);
        Encoder::runningRpmSum += Encoder::rpmSamples[i];
        delay(Encoder::SAMPLE_DELAY_MS);
    }
}

/**
 * @see
 * https://docs.arduino.cc/language-reference/en/functions/communication/serial/serialEvent/
 * @deprecated
 */
// void serialEvent() {
// Get position to write
// }
