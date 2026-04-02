/**
 * @file ActuatorDemo.ino
 * @brief Manually control the actuator
 * @version 1.1.1
 * @since Winter 2026
 * @author Noah (@BobSaidHi <https://github.com/bobsaidhi>) for
 * @CalPolyWindPower <https://github.com/calpolywindpower>
 * @author Inspired by Trevor (@rover-t <https://github.com/rover-t>) at
 * @CalPolyWindPower <https://github.com/calpolywindpower>
 *
 * The DFRobot FireBeetle 2 ESP32-C5 has one USB C port.  The baud rate is set
 * to to 115,000 below and uses the internal USB peripheral, not a separate
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
#include <Arduino.h>
#include <cstdint> // Fixed size integers
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
    constexpr int MIN_POS_us = 1000;    // Minimum position in microseconds
    constexpr int MAX_POS_us = 2000;    // Maximum position in microseconds
    constexpr int DEFAULT_us = 1500;    // Default position in microseconds
    constexpr int CONTROL_PIN = 3;      // PWM pin
    constexpr uint8_t FEEDBACK_PIN = 2; // Analog feedback pin
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

    Serial.println("Initalized 2026 Actuator Demo v2026-3-5 / v1.1.0");
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
    // Toggle LED because why not, but only on the EPS32
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
}

/**
 * @see
 * https://docs.arduino.cc/language-reference/en/functions/communication/serial/serialEvent/
 * @deprecated
 */
// void serialEvent() {
// Get position to write
// }
