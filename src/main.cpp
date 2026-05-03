// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java:
// https://pvs-studio.com

// System Includes
#include <esp_log.h>
#include <temperature_sensor.h>

// Library Includes
#include <Arduino.h>
// #include <PID_v1.h>

// Project Includes
#include "NacelleConfig.hpp"
// #include "2026Core/Net/Net-Application/Telnet.hpp"
// #include "2026Core/Net/NetAdapter_A.hpp"
// #include "2026Core/Net/NetAdapter_ FA.hpp"
#include "2026Core/CommonConfig.hpp" // Include after NacelleConfig due to macro precednece
#include "2026Core/Net/Net-Application/NTP.hpp"
// #include "2026Core/Net/Net-Application/OTA.hpp"
#include "2026Core/Net/Net-Link/AdapterESPNow.hpp"
// #include "2026Core/Net/Net-Link/AdapterUHCI.hpp" // NOSONAR
#include "2026Core/Net/Net-Phy/AdapterWLAN.hpp"
#include "2026Core/TurbinePacket/TurbinePacket.hpp"
#include "ActuonixL12.hpp"
#include "Encoder.hpp"
#include "NacelleComms.hpp"
#include "NacelleContainer.hpp"
#include "NacelleFSM.hpp"
#include "NacelleTasks.hpp"
#include "PID.hpp"

// MARK: Config
static constexpr const char *TAG = "NaMa";

// MARK: Function Prototypes

// Helper Functions
// ...

// MARK:  Global Objects

NacelleComms nacelleComms;
// AdapterWLAN adapterWLAN = AdapterWLAN();
// AdapterWLAN adapterWLAN;
// AdapterESPNow adapterESPNow = AdapterESPNow();
// AdapterESPNow adapterESPNow;
// SyncedClock netClock = SyncedClock(adapterESPNow); // todo
// SyncedClock netClock(adapterESPNow); // todo

ActuonixL12 pitchActuator(FR_FIREBEETLE2_ESP32C6::ACTUATOR_PWM_PIN,
                          PITCHING::SERVO_MIN_uS_2026,
                          PITCHING::SERVO_MAX_uS_2026);

PID pitchPIDController = PID( // todo
    {PITCHING::PITCH_Kp, PITCHING::PITCH_Ki, PITCHING::PITCH_Kd},
    /* Setpoint */ ENCODER::TARGET_RPM,
    PID::ProportionalMode::ProportionalOnMeas,
    {PITCHING::SERVO_MIN_uS_2026, PITCHING::SERVO_MAX_uS_2026},
    /* Min Sample Time */ 0,
    /* Direction */ PID::Direction::DIRECT, "PC");

NacelleContainer nacelle(pitchActuator, pitchPIDController, nacelleComms);
NacelleFSM nacelleFSM(nacelle);

/**
 * MARK: Setup
 * @details put your setup code here, to run once:
 */
void setup() {
    // Configure Hardware
    static bool serialInitialized = false;
    if (!serialInitialized) {
        Serial.begin(WTbCommonConfig::SERIAL_BAUD);
        ESP_LOGI(TAG, "Serial initialized");
        serialInitialized = true;
    }

    static bool LEDInitialized = false;
    if (!LEDInitialized) {
        pinMode(LED::PIN, OUTPUT); // Onboard LED
        digitalWrite(LED::PIN, HIGH);
        LEDInitialized = true;
        ESP_LOGI(TAG, "LED initialized");
    } else {
        // Save power during main operations
        digitalWrite(LED::PIN, LOW);
    }

    // Initialize Actuator
    static bool actuatorInitialized = false;
    if (!actuatorInitialized) {
        pitchActuator.begin();
        actuatorInitialized = true;
        ESP_LOGI(TAG, "Actuator Initialized");
    }

    // Configure Encoder
    static bool encoderInitialized = false;
    if (!encoderInitialized) {
        if (Encoder::initialize()) {
            ESP_LOGI(TAG, "Encoder initialized");
            encoderInitialized = true;
        } else {
            ESP_LOGE(TAG, "Failed to initialize encoder");
        }
    }

    // Configure New ESP-NOW + WiFI implementation

    // Configure WiFi
    // static bool wifiInitialized = false;
    // if (!wifiInitialized) {
    //     digitalWrite(LED::PIN,
    //                  LOW); // Will take a while, so turn off the LED
    //     uint8_t optimalChannel = adapterWLAN.identifyOptimalChannel();
    //     digitalWrite(LED::PIN, HIGH);
    //     ESP_LOGI(TAG, "Optimal WiFi Channel: %d", optimalChannel);
    //     if (adapterWLAN.begin(optimalChannel)) {
    //         ESP_LOGI(TAG, "WiFi initialized");
    //         wifiInitialized = true;
    //     } else {
    //         ESP_LOGE(TAG, "Failed to initialize WiFi");
    //     }
    // }
    // digitalWrite(LED::PIN, LOW);

    // Configure ESP-NOW
    static bool espNowInitalized = false;
    if (nacelleComms.begin()) {
        espNowInitalized = true;
    } else {
        ESP_LOGE(TAG, "Failed to initialize NacelleComms");
    }
    // if (!espNowInitalized) {
    //     if (adapterESPNow.begin()) {
    //         ESP_LOGI(TAG, "ESP-NOW initialized.");
    //         espNowInitalized = true;
    //     } else {
    //         ESP_LOGE(TAG, "Failed to initialize ESP-NOW");
    //     }
    // }
    // digitalWrite(LED::PIN, HIGH);

    // Configure ESP-NOW Peers
    // static bool peerRegistered = false;
    // if (!peerRegistered) {
    //     if (adapterESPNow.registerPeer(WTbNetConfig::LOAD_MAC)) {
    //         ESP_LOGI(TAG, "Registered peer");
    //         peerRegistered = true;
    //     } else {
    //         ESP_LOGE(TAG, "Failed to register peer");
    //     }
    // }
    // digitalWrite(LED::PIN, LOW);

    // Sync Time // FIXME! - Load accesses fault - DONE?
    // static bool timeSynced = false;
    // if (!timeSynced) {
    //     if (netClock.initTimeSync(WTbNetConfig::LOAD_MAC)) {
    //         ESP_LOGI(TAG, "Time sync initialized successfully");
    //         timeSynced = true;
    //     } else {
    //         ESP_LOGE(TAG, "Failed to initialize time sync");
    //     }
    // }
    // digitalWrite(LED::PIN, HIGH);

    // Print MAC Address // todo - verify
    // ESP_LOGI(
    //     TAG, "MAC Address: %s",
    // AdapterWLAN::formatMACAddress(adapterWLAN.getMACAddress()).c_str());
    // digitalWrite(LED::PIN, LOW);

    // TODO: Check ESP-NOW impl against last years
    // TODO: Configure response handler, load server

    digitalWrite(LED::PIN, LOW);

    // Set up tasks
    static bool tasksSetup = false;
    if (!tasksSetup) {
        for (TaskInfo &taskDesc : nacelle.mainTaskDescriptions) {
            ESP_LOGI(TAG, "Setting up task: %s", taskDesc.name);
            if (taskDesc.stackSize_bytes % sizeof(uint_fast8_t) != 0) {
                ESP_LOGW(TAG, "Stack size not word aligned");
            }
            // Syntax: xTaskCreate(Task function, Name of the task (for
            // debugging), Stack size (in words, not bytes), Task input
            // parameter, Priority of the task, Task handle)
            BaseType_t result = xTaskCreate(
                taskDesc.function, taskDesc.name, taskDesc.stackSize_bytes,
                taskDesc.pvParameters, taskDesc.priority, &(taskDesc.pxHandle));
            if (result != pdPASS) {
                ESP_LOGE(TAG, "Failed to create task %s", taskDesc.name);
            } else {
                taskDesc.initialized = true;
                ESP_LOGV(TAG,
                         "Created task %s with priority %u and stack "
                         "size %u bytes",
                         taskDesc.name, taskDesc.priority,
                         taskDesc.stackSize_bytes);
            }
        }

        NacelleFSM::UPDATE_RESULT result = nacelleFSM.updateState();
        if (result == NacelleFSM::UPDATE_RESULT::ERROR) {
            // Allegedly unreachable
            ESP_LOGE(TAG, "Error during FSM init., %d",
                     static_cast<uint_fast8_t>(result));
        } else if (result == NacelleFSM::UPDATE_RESULT::STATE_CHANGED) {
            // FIXME: Apparently this is always true
            ESP_LOGI(TAG, "Initialized FSM to state %d",
                     static_cast<uint_fast8_t>(nacelleFSM.getCurrentState()));
        } else if (result == NacelleFSM::UPDATE_RESULT::NO_CHANGE) {
            ESP_LOGE(TAG, "Failed to enter a valid state");
        } else {
            ESP_LOGE(TAG, "Unknown FSM init. result: %d",
                     static_cast<uint_fast8_t>(result));
        }

        tasksSetup = true;

        ESP_LOGI(TAG, "Setup complete!");
    }
}

/**
 * MARK: Main Tasks
 * @see
 * https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/01-Tasks-and-co-routines/05-Implementing-a-task
 * @see https://forum.arduino.cc/t/non-blocking-delay-actions/1044079
 * @see
 * https://docs.freertos.org/Documentation/02-Kernel/04-API-references/02-Task-control/03-xTaskDelayUntil
 */

/**
 * @SuppressWarnings("cpp:S5008") // Does not work
 */
[[noreturn]] void
vTaskUpdateFSM([[maybe_unused]] void *pvParameters) { // NOSONAR
    while (true) {
        static TickType_t xLastWakeTime = xTaskGetTickCount();

        NacelleFSM::UPDATE_RESULT result = nacelleFSM.updateState();
        if (result == NacelleFSM::UPDATE_RESULT::STATE_CHANGED) {
            ESP_LOGI(TAG, "FSM State Changed: %d",
                     nacelleFSM.getCurrentState());
        } else if (result == NacelleFSM::UPDATE_RESULT::ERROR) {
            // Allegedly unreachable
            ESP_LOGE(TAG, "Error updating FSM state");
        } else {
            // No change, nothing to log
        }

        BaseType_t xWasDelayed = xTaskDelayUntil(
            &xLastWakeTime, pdMS_TO_TICKS(RUN::TASK_INTERVALS::TI_FSM_mS));
        if (xWasDelayed != pdTRUE) {
            ESP_LOGE(TAG, "Timing not met!");
        }
    }
}

/**
 * @SuppressWarnings("cpp:S5008") // Does not work
 * TODO: PVS-Studio seems to think this can return and marking it no return
 * could result in undefind behavior
 */
[[noreturn]] void
vTaskPollSensors([[maybe_unused]] void *pvParameters) { // NOSONAR
    while (true) {
        static TickType_t xLastWakeTime = xTaskGetTickCount();
        static unsigned long previousTime = 0;
        unsigned long currentTime = micros();

        // Poll encoder & update stored value
        // todo: back off and retry if not working
        int_fast16_t lastRPM = nacelle.currentRPM;
        Encoder::getRpmMovingAverage(nacelle.currentRPM);
        int_fast16_t deltaRPM = nacelle.currentRPM - lastRPM;
        int_fast32_t deltaTime_us = currentTime - previousTime;
        constexpr unsigned long m_TO_BASE = 1000;
        constexpr unsigned long u_TO_m = 1000;
        constexpr unsigned long u_TO_BASE = m_TO_BASE * u_TO_m;
        nacelle.angularAccell_RPMPS = deltaRPM * u_TO_BASE / deltaTime_us;
        previousTime = currentTime;

        BaseType_t xWasDelayed = xTaskDelayUntil(
            &xLastWakeTime,
            pdMS_TO_TICKS(RUN::TASK_INTERVALS::TI_POLL_SENSORS_mS));
        if (xWasDelayed != pdTRUE) {
            ESP_LOGE(TAG, "Timing not met!");
        }
    }
}

/**
 * @SuppressWarnings("cpp:S5008") // Does not work
 */
[[noreturn]] void vTaskPitch([[maybe_unused]] void *pvParameters) { // NOSONAR
    while (true) {
        static TickType_t xLastWakeTime = xTaskGetTickCount();

        if (nacelleFSM.getCurrentState() == FSMCommon::States::sStartLoad ||
            nacelleFSM.getCurrentState() == FSMCommon::States::sRunLoad) {
            // Fine
        } else if (nacelleFSM.getCurrentState() ==
                   FSMCommon::States::sCurtail) {
            auto pidOutput = static_cast<uint_fast16_t>(
                pitchPIDController.compute(nacelle.currentRPM)); // todo - input
            ESP_LOGI(TAG, "PID Output: %u", pidOutput);
            pitchActuator.writePosMicros(pidOutput);
        } else {
            // No updates needed
            ESP_LOGI(TAG, "No pitch updates needed in state %d, supsending",
                     nacelleFSM.getCurrentState());
            // DONE: Consider suspending when not in use
            vTaskSuspend(nacelle
                             .mainTaskDescriptions
                                 [NacelleContainer::MAIN_TASK_IDS::TID_PITCH]
                             .pxHandle); // Suspend until reenabled by FSM
        }

        BaseType_t xWasDelayed = xTaskDelayUntil(
            &xLastWakeTime, pdMS_TO_TICKS(RUN::TASK_INTERVALS::TI_PITCH_mS));
        if (xWasDelayed != pdTRUE) {
            ESP_LOGE(TAG, "Timing not met!");
        }
    }
}

// MARK: Network Tasks

/**
 * @SuppressWarnings("cpp:S5008") // Does not work
 */
[[noreturn]] void
vTaskRecvData([[maybe_unused]] void *pvParameters) { // NOSONAR
    while (true) {
        static TickType_t xLastWakeTime = xTaskGetTickCount();

        if (true) { // todo - when to suspend?
            LoadboxPacket packet;
            if (xQueueReceive(NacelleComms::priorityDataQueue, &packet, 0) ==
                pdPASS) {
                ESP_LOGV(TAG, "Received packet: safety=%u", packet.safety);
                nacelle.d_mVPS = packet.d_mVPS;
                nacelle.current_mA = packet.current_mA;
                nacelle.dIPS = packet.dIPS;
                nacelle.setSafetyFlag(
                    static_cast<ESTOP_TYPE_FAST>(packet.safety));
            }

            BaseType_t xWasDelayed = xTaskDelayUntil(
                &xLastWakeTime, pdMS_TO_TICKS(RUN::TASK_INTERVALS::TI_RECV_ms));
            if (xWasDelayed != pdTRUE) {
                ESP_LOGE(TAG, "Timing not met!");
            }
        } else {
            // Suspend until reenabled from interrupt
            // vTaskSuspend(mainTaskDescriptions[MAIN_TASK_IDS::TID_RECV].pxHandle);
            BaseType_t xWasDelayed = xTaskDelayUntil( // TODO: Fix polling
                &xLastWakeTime, pdMS_TO_TICKS(RUN::TASK_INTERVALS::TI_RECV_ms));
            if (xWasDelayed != pdTRUE) {
                ESP_LOGE(TAG, "Timing not met!");
            }

            // (suspend ^ blocks
            // setup)
        }
    }
}

/**
 * @brief Task to handle outbound data
 * @SuppressWarnings("cpp:S5008") // Does not work
 */
[[noreturn]] void
vTaskSendData([[maybe_unused]] void *pvParameters) { // NOSONAR
    while (true) {
        static TickType_t xLastWakeTime = xTaskGetTickCount();

        if (true) { // todo - when to suspend?
            (void)nacelleComms.sendNacelleData(
                static_cast<int16_t>(nacelle.currentRPM),
                static_cast<int16_t>(nacelle.angularAccell_RPMPS));
            BaseType_t xWasDelayed = xTaskDelayUntil(
                &xLastWakeTime, pdMS_TO_TICKS(RUN::TASK_INTERVALS::TI_SEND_ms));
            if (xWasDelayed != pdTRUE) {
                ESP_LOGE(TAG, "Timing not met!");
            }
        } else {
            // Suspend until reenabled
            // vTaskSuspend(mainTaskDescriptions[MAIN_TASK_IDS::TID_SEND].pxHandle);
            BaseType_t xWasDelayed = xTaskDelayUntil( // TODO: Fix polling
                &xLastWakeTime, pdMS_TO_TICKS(RUN::TASK_INTERVALS::TI_SEND_ms));
            if (xWasDelayed != pdTRUE) {
                ESP_LOGE(TAG, "Timing not met!");
            }

            // (suspend ^ blocks
            // setup)
        }
    }
}

// MARK: Utility Tasks

/**
 * @SuppressWarnings("cpp:S5008") // Does not work
 */
[[noreturn]] void
vTaskConfigure([[maybe_unused]] void *pvParameters) { // NOSONAR
    while (true) {
        static TickType_t xLastWakeTime = xTaskGetTickCount();

        // setup(); // todo
        BaseType_t xWasDelayed = xTaskDelayUntil(
            &xLastWakeTime, pdMS_TO_TICKS(RUN::TASK_INTERVALS::TI_CFG_ms));
        if (xWasDelayed != pdTRUE) {
            ESP_LOGE(TAG, "Timing not met!");
        }
    }
}

/**
 * @SuppressWarnings("cpp:S5008") // Does not work
 */
[[noreturn]] void vTaskTelnet([[maybe_unused]] void *pvParameters) { // NOSONAR
    while (true) {
        static TickType_t xLastWakeTime = xTaskGetTickCount();

        // TELNET::loop();
        BaseType_t xWasDelayed = xTaskDelayUntil(
            &xLastWakeTime, pdMS_TO_TICKS(RUN::TASK_INTERVALS::TI_TELNET_ms));
        if (xWasDelayed != pdTRUE) {
            ESP_LOGE(TAG, "Timing not met!");
        }
    }
}

/**
 * @SuppressWarnings("cpp:S5008") // Does not work
 */
[[noreturn]] void vTaskOTA([[maybe_unused]] void *pvParameters) { // NOSONAR
    while (true) {
        static TickType_t xLastWakeTime = xTaskGetTickCount();

        BaseType_t xWasDelayed = xTaskDelayUntil(
            &xLastWakeTime, pdMS_TO_TICKS(RUN::TASK_INTERVALS::TI_OTA_ms));
        if (xWasDelayed != pdTRUE) {
            ESP_LOGE(TAG, "Timing not met!");
        }
    }
}

// MARK: Status Tasks

/**
 * @SuppressWarnings("cpp:S5008") // Does not work
 */
[[noreturn]] void
vTaskStatusLED([[maybe_unused]] void *pvParameters) { // NOSONAR
    while (true) {
        static TickType_t xLastWakeTime = xTaskGetTickCount();

        ESP_LOGV(TAG, "vTSL");
        digitalWrite(LED::PIN, HIGH);

        BaseType_t xWasDelayed = xTaskDelayUntil(
            &xLastWakeTime, pdMS_TO_TICKS(LED::BLINK_ON_MILLIS));
        if (xWasDelayed != pdTRUE) {
            ESP_LOGE(TAG, "Timing not met!");
        }

        digitalWrite(LED::PIN, LOW);

        xWasDelayed = xTaskDelayUntil(&xLastWakeTime,
                                      pdMS_TO_TICKS(LED::BLINK_OFF_MILLIS));
        if (xWasDelayed != pdTRUE) {
            ESP_LOGE(TAG, "Timing not met!");
        }
    }
}

/**
 * @brief Convert Celsius to Fahrenheit
 */
// consteval uint_fast8_t celsiusToFahrenheit(uint_fast8_t celsius) {
//     return (celsius * 9 / 5) + 32;
// }
// static_assert(celsiusToFahrenheit(0) == 32);
// static_assert(celsiusToFahrenheit(130) <= UINT8_MAX,
//               "Value exceeds uint8_t max");

/**
 * @brief Convert Fahrenheit to Celsius
 */
// consteval uint_fast8_t fahrenheitToCelsius(uint_fast8_t fahrenheit) {
//     return (fahrenheit - 32) * 5 / 9;
// }
constexpr uint32_t ITEMS_TO_LOG = 4;
constexpr uint32_t LOG_ITEM_INTERVAL_MS =
    RUN::TASK_INTERVALS::TI_LOG_DATA_ms / ITEMS_TO_LOG;
/**
 * @SuppressWarnings("cpp:S5008") // Does not work
 */
[[noreturn]] void vTaskLogData([[maybe_unused]] void *pvParameters) { // NOSONAR
    /**
     * @See
     * https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32c5/api-reference/peripherals/temp_sensor.html
     * TODO: The temp. sensor may use more power
     * Not needed?: Temp interput/ callback/ hardware monitoring
     */
    temperature_sensor_handle_t tempSensHandle = NULL;
    temperature_sensor_config_t tempSensConfig =
        TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 100);
    ESP_ERROR_CHECK(
        temperature_sensor_install(&tempSensConfig, &tempSensHandle));

    while (true) {
        static TickType_t xLastWakeTime = xTaskGetTickCount();

        // if (!Serial.isConnected()) {
        //     BaseType_t xWasDelayed = xTaskDelayUntil(
        //         &xLastWakeTime, pdMS_TO_TICKS(LOG_ITEM_INTERVAL_MS));
        //     if (xWasDelayed != pdTRUE) {
        //         ESP_LOGE(TAG, "Timing not met!");
        //     }
        //     continue;
        // }

        ESP_LOGD(TAG, "Logging Data:");

        // ESP_LOGI(TAG, "Time: %llu", SyncedClock::getSystemTimer());

        // Free RTOS Stats
        ESP_LOGI(TAG, "Num tasks reported by FreeRTOS: %u",
                 uxTaskGetNumberOfTasks());

        constexpr uint_fast8_t REC_BYTES_PER_TASK = 40;
        constexpr uint_fast8_t NUM_ESP_TASKS = 8;
        constexpr uint_fast16_t STATS_BUFFER_SIZE =
            REC_BYTES_PER_TASK * (NUM_MAIN_TASKS + NUM_ESP_TASKS);
        // char statsBuffer[STATS_BUFFER_SIZE] = {'\0'}; // Frowned on
        // by sonar lint:
        etl::string<STATS_BUFFER_SIZE> statsBuffer = {'\0'};
        if (uxTaskGetNumberOfTasks() > NUM_MAIN_TASKS + NUM_ESP_TASKS) {
            ESP_LOGE(TAG,
                     "Number of tasks (%d) exceeds expected max (%d), "
                     "skipping to "
                     "prevent memory corruption",
                     uxTaskGetNumberOfTasks(), NUM_MAIN_TASKS + NUM_ESP_TASKS);
        } else {
            // TODO: Not recommended in production
            vTaskGetRunTimeStats(statsBuffer.data());
            statsBuffer[STATS_BUFFER_SIZE - 1] =
                '\0'; // hard cap, avoid over-read
            statsBuffer.trim_to_terminator();
            // Alt. realted function: uxTaskGetSystemState();
            ESP_LOGI(TAG, "Task Run Time Stats:\n%s", statsBuffer.c_str());
            delay(LOG_ITEM_INTERVAL_MS);

            for (TaskInfo &taskDesc : nacelle.mainTaskDescriptions) {
                taskDesc.minFreeStack_Bytes =
                    uxTaskGetStackHighWaterMark(taskDesc.pxHandle);
                ESP_LOGI(TAG, "T: %s, U: %u, F: %u", taskDesc.name,
                         taskDesc.stackSize_bytes - taskDesc.minFreeStack_Bytes,
                         taskDesc.minFreeStack_Bytes);
            }
            for (TaskInfo &taskDesc : nacelle.optionalTaskDescriptions) {
                if (taskDesc.initialized) { // Not sure if you can get stack
                                            // info for uninitialized tasks,
                                            // so check first
                    taskDesc.minFreeStack_Bytes =
                        uxTaskGetStackHighWaterMark(taskDesc.pxHandle);
                    ESP_LOGI(TAG, "T: %s, U: %u, F: %u", taskDesc.name,
                             taskDesc.stackSize_bytes -
                                 taskDesc.minFreeStack_Bytes,
                             taskDesc.minFreeStack_Bytes);
                }
            }
        }
        delay(LOG_ITEM_INTERVAL_MS);

        ESP_LOGI(TAG, "Minimum free heap: %u bytes",
                 esp_get_minimum_free_heap_size());
        delay(LOG_ITEM_INTERVAL_MS);

        // Enable temperature sensor
        ESP_ERROR_CHECK(temperature_sensor_enable(tempSensHandle));
        // Get converted sensor data
        float tempSens_out;
        ESP_ERROR_CHECK(
            temperature_sensor_get_celsius(tempSensHandle, &tempSens_out));
        auto tempTrunc_C = static_cast<int32_t>(tempSens_out);
        constexpr int32_t MAX_EXT_TEMP = 105;
        constexpr int32_t MIN_EXT_TEMP = -40;
        if (tempTrunc_C > MAX_EXT_TEMP || tempTrunc_C < MIN_EXT_TEMP) {
            ESP_LOGE(TAG, "Temperature out of bounds: %d dC", tempTrunc_C);
        } else {
            ESP_LOGI(TAG, "Temperature: %d dC", tempTrunc_C);
        }
        // Disable the temperature sensor if it is not needed and save
        // the power
        ESP_ERROR_CHECK(temperature_sensor_disable(tempSensHandle));
        delay(LOG_ITEM_INTERVAL_MS);

        // esp_wifi_get_bandwidth
        // esp_wifi_sta_get_rssi

        // ESP_LOGI(TAG, "%s", netClock.getLogString().c_str());
        // delay(LOG_ITEM_INTERVAL_MS);

        ESP_LOGI(TAG, "%s", pitchActuator.getLogString().c_str());
        delay(LOG_ITEM_INTERVAL_MS);

        ESP_LOGI(TAG, "%s", pitchPIDController.getLogString().c_str());
        delay(LOG_ITEM_INTERVAL_MS);

        ESP_LOGI(TAG, "%s", nacelle.getLogString().c_str());
        delay(LOG_ITEM_INTERVAL_MS);

        ESP_LOGI(TAG, "Current State: %d", nacelleFSM.getCurrentState());
        delay(LOG_ITEM_INTERVAL_MS);

        ESP_LOGI(TAG, "%s", Encoder::getLogString().c_str());
        delay(LOG_ITEM_INTERVAL_MS);

        static unsigned int prevTime_us = 0;
        static NacelleComms::LogData lastLogData = {0};
        unsigned int currentTime_us = micros();
        ESP_LOGI(TAG, "%s", nacelleComms.getLogString().c_str());
        ESP_LOGI(TAG, "%s", nacelleComms.getLogString().c_str());
        NacelleComms::LogData currentLogData = nacelleComms.getLogData();
        unsigned long deltaTime_us = currentTime_us - prevTime_us;
        uint_fast32_t deltaTxEvents =
            currentLogData.txEvents - lastLogData.txEvents;
        uint_fast32_t deltaBytesSent =
            currentLogData.bytesSent - lastLogData.bytesSent;
        uint_fast32_t deltaBytesFailed =
            currentLogData.bytesNotSent - lastLogData.bytesNotSent;
        uint_fast32_t deltaRxEvents =
            currentLogData.rxEvents - lastLogData.rxEvents;
        uint_fast32_t deltaBytesReceived =
            currentLogData.bytesReceived - lastLogData.bytesReceived;

        constexpr unsigned long m_TO_BASE = 1000;
        constexpr unsigned long u_TO_m = 1000;
        constexpr unsigned long u_TO_BASE = m_TO_BASE * u_TO_m;
        ESP_LOGI(TAG, "TxE/s: %u, TxBS/s: %u, TxBF/s: %u, RxE/s: %u, RxB/s: %u",
                 deltaTxEvents * u_TO_BASE / deltaTime_us,
                 deltaBytesSent * u_TO_BASE / deltaTime_us,
                 deltaBytesFailed * u_TO_BASE / deltaTime_us,
                 deltaRxEvents * u_TO_BASE / deltaTime_us,
                 deltaBytesReceived * u_TO_BASE / deltaTime_us);
        prevTime_us = currentTime_us;
        lastLogData = currentLogData;

        prevTime_us = currentTime_us;

        // BaseType_t xWasDelayed = xTaskDelayUntil(
        //     &xLastWakeTime, pdMS_TO_TICKS(LOG_ITEM_INTERVAL_MS));
        // if (xWasDelayed != pdTRUE) {
        //     ESP_LOGE(TAG, "Timing not met!");
        // }
    }
}

/**
 * MARK: loop
 * Arduino: put your main code here, to run repeatedly:
 */
void loop() { delay(1000); }
