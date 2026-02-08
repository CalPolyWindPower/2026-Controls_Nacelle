// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java:
// https://pvs-studio.com

/* Includes */
// #include "2026Core/Net/Net-Link/AdapterUHCI.hpp"
// #include "2026Core/Net/NetAdapter_A.hpp"
#include "CommonConfig.hpp"
#include <Arduino.h>
// #include "2026Core/Net/Net-Application/NTP.hpp"
// #include "2026Core/Net/NetAdapter_A.hpp"
#include "2026Core/Net/Net-Link/AdapterESPNow.hpp"

#include <esp_log.h>

/* Config */
static constexpr const char *TAG = "NaMa";

/* Function Prototypes */
void vTaskStatusLED(void *pvParameters);
void vTaskConfigure(void *pvParameters);

/* Global Objects */
AdapterESPNow adapterESPNow = AdapterESPNow();
// SyncedClock netClock = SyncedClock(adapterESPNow); // todo

/**
 * MARK: Setup
 * put your setup code here, to run once:
 */
void setup() {
    Serial.begin(115200);
    ESP_LOGI(TAG, "Serial initialized");
    // Serial.println("Hello world!");

    pinMode(LED::LED_PIN, OUTPUT);

    // Set up tasks
    xTaskCreate(vTaskStatusLED, // Task function
                "Status LED",   // Name of the task (for debugging)
                1024,           // Stack size (in words, not bytes)
                nullptr,        // Task input parameter
                1,              // Priority of the task
                nullptr         // Task handle
    );
    xTaskCreate(vTaskConfigure, "Cfg", 1024, nullptr, 14, nullptr);
    // TODO: Note: Task priority must be <25
}

/**
 * MARK: Tasks
 * @see
 * https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/01-Tasks-and-co-routines/05-Implementing-a-task
 * @see https://forum.arduino.cc/t/non-blocking-delay-actions/1044079
 */

void vTaskStatusLED(void *pvParameters) {
    while (true) {
        digitalWrite(LED::LED_PIN, HIGH);
        delay(LED::BLINK_ON_MILLIS);
        digitalWrite(LED::LED_PIN, LOW);
        delay(LED::BLINK_OFF_MILLIS);
    }
}

void vTaskConfigure(void *pvParameters) {
    while (true) {
        delay(1000);
    }
}

/**
 * @brief Task to handle Telnet connections
 */
void vTaskTelnet(void *pvParameters) {
    while (true) {
        delay(1000);
    }
}

/**
 * @brief Task to handle ElegantOTA connections
 * @deprecated Just use a USB cable if possible
 */
void vTaskOTA(void *pvParameters) {
    while (true) {
        delay(1000);
    }
}

/**
 * @brief Task to poll high priority sensors
 */
void vTaskPollSensors(void *pvParameters) {
    while (true) {
        delay(1000);
    }
}

/**
 * @brief Task to control the pitch actuator
 */
void vTaskPitch(void *pvParameters) {
    while (true) {
        delay(1000);
    }
}

/**
 * @brief Task to handle inbound data that has been queued
 */
void vTaskHandleInboundData(void *pvParameters) {
    while (true) {
        delay(1000);
    }
}

/**
 * @brief Task to handle outbound data that has been queued
 */
void vTaskHandleOutboundData(void *pvParameters) {
    while (true) {
        delay(1000);
    }
}

/**
 * @brief Task to log data
 */
void vTaskLogData(void *pvParameters) {
    while (true) {
        delay(1000);
    }
}

/**
 * @brief Task to track idle time
 */
void vTaskIdle(void *pvParameters) {
    while (true) {
        delay(1000);
    }
}

/**
 * MARK: loop
 * Arduino: put your main code here, to run repeatedly:
 */
void loop() {
    // ESP_LOGI(TAG, "Time: %llu", SyncedClock::getSystemTimer());
    delay(1000);
}
