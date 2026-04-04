#pragma once

// Includes
#include <cstdint>
#include <freertos/projdefs.h>
#include <freertos/task.h>
#include <portmacro.h>

// MARK: Datatypes * Co
struct TaskInfo {
    const TaskFunction_t function;
    const char *const name;
    const configSTACK_DEPTH_TYPE stackSize_bytes = 1024;
    void *const pvParameters = nullptr;
    const UBaseType_t priority; // Note: Task priority must be <25 for some
                                // reason, possible bug
    TaskHandle_t pxHandle = nullptr;
    UBaseType_t minFreeStack_Bytes = 0;
    bool initialized = false;
};

// MARK: Constants
constexpr uint_fast8_t NUM_MAIN_TASKS = 8;     // Must match number of entires!
constexpr uint_fast8_t NUM_OPTIONAL_TASKS = 2; // Must match number of entires!

// MARK: Function Prototypes
// Main Tasks
/**
 * @brief Task to control run the FSM
 */
void vTaskUpdateFSM([[maybe_unused]] void *pvParameters);
/**
 * @brief Task to poll high priority sensors
 */
void vTaskPollSensors([[maybe_unused]] void *pvParameters);
/**
 * @brief Task to control the pitch actuator
 */
void vTaskPitch([[maybe_unused]] void *pvParameters);
/**
 * @brief Task to handle inbound data that has been queued
 */
void vTaskRecvData([[maybe_unused]] void *pvParameters);

/**
 * @brief Task to handle outbound data that has been queued
 */
void vTaskSendData([[maybe_unused]] void *pvParameters);
void vTaskConfigure([[maybe_unused]] void *pvParameters);
void vTaskStatusLED([[maybe_unused]] void *pvParameters);
/**
 * @brief Task to log data
 */
void vTaskLogData([[maybe_unused]] void *pvParameters);

// Optional Tasks
/**
 * @brief Task to handle Telnet connections
 * @deprecated Just use a USB cable if possible
 */
void vTaskTelnet([[maybe_unused]] void *pvParameters);
/**
 * @brief Task to handle ElegantOTA connections
 * @deprecated Just use a USB cable if possible
 */
void vTaskOTA([[maybe_unused]] void *pvParameters);
