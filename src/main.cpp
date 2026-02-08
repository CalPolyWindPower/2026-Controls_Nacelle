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
void vTaskLogData(void *pvParameters);

/* Global Objects */
struct TaskInfo {
    const TaskFunction_t function;
    const char *const name;
    const configSTACK_DEPTH_TYPE stackSize_bytes = 1024;
    void *const pvParameters = nullptr;
    const UBaseType_t priority; // Note: Task priority must be <25 for some
                                // reason, possible bug
    TaskHandle_t pxCreatedTask = nullptr;
    UBaseType_t minFreeStack_Bytes = 0;
};
constexpr uint_fast8_t NUM_TASKS = 3;
etl::array<TaskInfo, NUM_TASKS> taskDescriptions = {
    TaskInfo{vTaskStatusLED, "LED", 256, nullptr, 1, nullptr, 0},
    TaskInfo{vTaskConfigure, "Cfg", 256, nullptr, 14, nullptr, 0},
    TaskInfo{vTaskLogData, "Log", 2056, nullptr, 0, nullptr, 0}};
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
    for (TaskInfo &taskDesc : taskDescriptions) {
        if (taskDesc.stackSize_bytes % sizeof(uint_fast8_t) != 0) {
            ESP_LOGW(TAG, "Stack size not word aligned");
        }
        // Syntax: xTaskCreate(Task function, Name of the task (for debugging),
        // Stack size (in words, not bytes), Task input parameter, Priority of
        // the task, Task handle)
        BaseType_t result =
            xTaskCreate(taskDesc.function, taskDesc.name,
                        taskDesc.stackSize_bytes, taskDesc.pvParameters,
                        taskDesc.priority, &(taskDesc.pxCreatedTask));
        if (result != pdPASS) {
            ESP_LOGE(TAG, "Failed to create task %s", taskDesc.name);
        } else {
            ESP_LOGV(
                TAG, "Created task %s with priority %u and stack size %u bytes",
                taskDesc.name, taskDesc.priority, taskDesc.stackSize_bytes);
        }
    }
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

constexpr uint32_t LOG_INTERVAL_MS = 1000;
constexpr uint32_t ITEMS_TO_LOG = 1;
/**
 * @brief Task to log data
 */
void vTaskLogData(void *pvParameters) {
    while (true) {
        // if (!Serial.isConnected()) {
        //     delay(LOG_INTERVAL_MS);
        //     continue;
        // }

        ESP_LOGV(TAG, "Logging Data:");

        for (TaskInfo &taskDesc : taskDescriptions) {
            taskDesc.minFreeStack_Bytes =
                uxTaskGetStackHighWaterMark(taskDesc.pxCreatedTask);
            ESP_LOGI(TAG, "T: %s, U: %u, F: %u", taskDesc.name,
                     taskDesc.stackSize_bytes - taskDesc.minFreeStack_Bytes,
                     taskDesc.minFreeStack_Bytes);
        }
        delay(LOG_INTERVAL_MS / ITEMS_TO_LOG);
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
