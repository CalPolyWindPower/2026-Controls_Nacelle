/**
 * @file ActuatorDemo.ino
 * @brief Manually control the actuator
 *
 * The DFRobot Firebeetle 2 ESP32-C5 has one USB C port.  The baud
 *   rate is seto to 115,000 below and uses the internal USB
 *   perheprial, not a seperate chip.
 * Some compatible serial monitors include the Arduino IDE
 *   <https://www.arduino.cc/en/software/>, VSCode "Serial" extension
 *   from Microsoft
 *   <https://marketplace.visualstudio.com/items?itemName=ms-vscode.vscode-serial-monitor>,
 *   or PuTTY.
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
 *     (See also: https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html)
 * 3. Go to the library manager in the left sidebar and install "ESP32Servo" by Kevin.
 * 4. Load the attached sketch or the latest version from GitHub:
 *    https://github.com/CalPolyWindPower/2026-Controls_Nacelle/blob/main/demos/ActuatorDemo/ActuatorDemo.ino
 * 5. Set the board to "ESP32C5 Dev Module".  There may be multiple
 *    COM / serial ports to choose from.  If one is labled "ESP32
 *    Family Device", use that and switch it to "ESP32C5 Dev Module".
 * 6. Under Tools, set "USB CDC On Boot" to "Enabled"
 * 7. [Recomended] Under Tools, set "Core Debug Level" to "Info"
 * 8. Select Upload
 * 9. Open a serial terminal
 *   A. In Arduino IDE, got Tools > Serial Monitor and switch to 115200 baud
 *
 * @author Noah (@BobSaidHi <https://github.com/bobsaidhi>) for
 *   Cal Poly Wind Power (@calpolywindpower <https://github.com/calpolywindpower>)
 * @see Inspired by
 *   https://github.com/rover-t/CPWP2024-Controls/blob/main/ControlCode/OpenLoopController/OpenLoopController.ino
 * @see Inspired by
 *   https://github.com/rover-t/CPWP2024-Controls/blob/main/ControlCode/OpenLoopController_alt/OpenLoopController_alt.ino
 */

// Imports
#include <Arduino.h>
#include <cstdint>  // Fixed size integers
#if defined(ESP32)
/**
 * @see https://github.com/madhephaestus/ESP32Servo/blob/master/src/ESP32Servo.h
 */
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif

// Varialbes
namespace SERIAL_CONFIG {
constexpr long BAUD = 115200;
}  // namespace serial

namespace Actuator {
Servo device = Servo();
constexpr int MIN_POS_us = 1000;     // Minimum position in microseconds
constexpr int MAX_POS_us = 2000;     // Maximum position in microseconds
constexpr int DEFAULT_us = 1500;     // Default position in microseconds
constexpr int CONTROL_PIN = 3;       // PWM pin
constexpr uint8_t FEEDBACK_PIN = 2;  // Analgo feedback pin
}  // namespace Actuator

namespace LED {
constexpr uint8_t PIN = 15;
constexpr uint_fast32_t TIME_ON_MS = 1000;
constexpr uint_fast32_t TIME_OFF_MS = 3000;
}  // namespace LED

/**
 * @details put your setup code here, to run once:
 */
void setup() {
  Serial.begin(SERIAL_CONFIG::BAUD);  // Start serial

// The DFRobot FireBeetle 2 ESP32-C5 has one user controllable LED built in
#if defined(ESP32)
  /**
   * @see https://docs.arduino.cc/language-reference/en/functions/digital-io/pinMode/
   */
  pinMode(LED::PIN, OUTPUT);     // Setup LED
  digitalWrite(LED::PIN, HIGH);  // Toggle LED

  // Debug actuator pin
  // pinMode(Actuator::CONTROL_PIN, OUTPUT);     // Setup LED
  // digitalWrite(Actuator::CONTROL_PIN, HIGH);  // Toggle LED
  // while(1) {}
#endif

  // Servo device = Servo();

  Actuator::device.attach(Actuator::CONTROL_PIN, Actuator::MIN_POS_us, Actuator::MAX_POS_us);  // Attach actuator
  Actuator::device.writeMicroseconds(Actuator::DEFAULT_us);

  Serial.println("Initalized 2026 Actuator Demo v2026-3-5 / v1.0.0");
  digitalWrite(LED::PIN, LOW);  // Toggle LED

  Serial.print("Input servo position in Microseoncds: ");
}

/**
 * @details put your main code here, to run repeatedly:
 */
void loop() {
  // Check if Serial data is avaiable
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
  // Toggle LED because why not
  static int lastLEDTime_ms = 0;  // Track time toggled
  static bool LEDOn = false;      // Track LED state

  // Check LED state and update
  if (!LEDOn && millis() - lastLEDTime_ms > LED::TIME_OFF_MS) {
    digitalWrite(LED::PIN, HIGH);  // Toggle LED
    lastLEDTime_ms = millis();     // Save time
    LEDOn = true;
  } else if (LEDOn && millis() - lastLEDTime_ms > LED::TIME_ON_MS) {
    digitalWrite(LED::PIN, LOW);  // Toggle LED
    lastLEDTime_ms = millis();    // Save time
    LEDOn = false;
  }
#endif
}

/**
 * @see https://docs.arduino.cc/language-reference/en/functions/communication/serial/serialEvent/
 * @deprecated
 */
// void serialEvent() {
// Get position to write
// }
