// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java:
// https://pvs-studio.com

/* Includes */
#include "NacelleConfig.hpp"
// #include "2026Core/Net/Net-Application/Telnet.hpp"
// #include "2026Core/Net/Net-Link/AdapterUHCI.hpp"
// #include "2026Core/Net/NetAdapter_A.hpp"
// #include "2026Core/Net/NetAdapter_A.hpp"
// #include <PID.hpp>
#include "2026Core/CommonConfig.hpp" // Include after NacelleConfig due to macro precednece
#include "2026Core/Net/Net-Application/NTP.hpp"
// #include "2026Core/Net/Net-Application/OTA.hpp"
#include "2026Core/Net/Net-Link/AdapterESPNow.hpp"
#include "2026Core/Net/Net-Phy/AdapterWLAN.hpp"
#include "NacelleContainer.hpp"
// #include "NacelleFSM.hpp"
#include "Tasks.hpp"
#include <ActuonixL12.hpp>
#include <Arduino.h>
#include <NacelleFSM.hpp>
#include <esp_log.h>
#include <temperature_sensor.h>
// MARK: Config
static constexpr const char *TAG = "NaMa";

// MARK: Function Prototypes

// Helper Functions
// ...

// MARK:  Global Objects

// AdapterWLAN adapterWLAN = AdapterWLAN();
AdapterWLAN adapterWLAN;
// AdapterESPNow adapterESPNow = AdapterESPNow();
AdapterESPNow adapterESPNow;
// SyncedClock netClock = SyncedClock(adapterESPNow); // todo
SyncedClock netClock(adapterESPNow); // todo

// PID pitchPIDController =
//     PID(PITCHING::PITCH_Kp, PITCHING::PITCH_Ki, PITCHING::PITCH_Kd,
//         // PITCHING::TARGET_RPM,
//         2000.0f, PID::ProportionalMode::ProportionalOnMeas,
//         // PITCH_MAX_ANGLE_DEG = min actuator extension = minimum PID output
//         //
//         (float)pitchActuator.angleToMicros(WTbNacCfg::PITCH_MAX_ANGLE_DEG),
//         1000.0f,
//         // PITCH_CUTIN_ANGLE_DEG = max actuator extension = maximum PID
//         output
//         //
//         (float)pitchActuator.angleToMicros(WTbNacCfg::PITCH_MIN_ANGLE_DEG),
//         2000.0f, 0, PID::Direction::DIRECT, "PC");
// ; // todo

ActuonixL12 pitchActuator(FR_FIREBEETLE2_ESP32C6::ACTUATOR_PWM_PIN,
                          PITCHING::SERVO_MIN_uS_2026,
                          PITCHING::SERVO_MAX_uS_2026);

NacelleContainer nacelle(pitchActuator);
NacelleFSM nacelleFSM(nacelle);

/**
 * MARK: Setup
 * @details put your setup code here, to run once:
 */
void setup() {
    static bool serialInitialized = false;
    if (!serialInitialized) {
        Serial.begin(115200);
        ESP_LOGI(TAG, "Serial initialized");
        serialInitialized = true;
    }

    // Configure Hardare
    static bool LEDInitialized = false;
    if (!LEDInitialized) {
        pinMode(LED::LED_PIN, OUTPUT); // Onboard LED
        digitalWrite(LED::LED_PIN, HIGH);
        LEDInitialized = true;
        ESP_LOGI(TAG, "LED initialized");
    } else {
        // Save power during main operations
        digitalWrite(LED::LED_PIN, LOW);
    }

    // Initialize Actuator
    static bool actuatorInitialized = false;
    if (!actuatorInitialized) {
        pitchActuator.begin();
        actuatorInitialized = true;
        ESP_LOGI(TAG, "Actuator Initialized");
    }

    // Configure WiFi
    static bool wifiInitialized = false;
    if (!wifiInitialized) {
        digitalWrite(LED::LED_PIN,
                     LOW); // Will take a while, so turn off the LED
        uint8_t optimalChannel = adapterWLAN.identifyOptimalChannel();
        digitalWrite(LED::LED_PIN, HIGH);
        ESP_LOGI(TAG, "Optimal WiFi Channel: %d", optimalChannel);
        if (adapterWLAN.begin(optimalChannel)) {
            ESP_LOGI(TAG, "WiFi initialized");
            wifiInitialized = true;
        } else {
            ESP_LOGE(TAG, "Failed to initialize WiFi");
        }
    }
    digitalWrite(LED::LED_PIN, LOW);

    // Configure ESP-NOW
    static bool espNowInitalized = false;
    if (!espNowInitalized) {
        if (adapterESPNow.begin()) {
            ESP_LOGI(TAG, "ESP-NOW initialized.");
            espNowInitalized = true;
        } else {
            ESP_LOGE(TAG, "Failed to initialize ESP-NOW");
        }
    }
    digitalWrite(LED::LED_PIN, HIGH);

    // Configure ESP-NOW Peers
    static bool peerRegistered = false;
    if (!peerRegistered) {
        if (adapterESPNow.registerPeer(WTbNetConfig::LOAD_MAC)) {
            ESP_LOGI(TAG, "Registered peer");
            peerRegistered = true;
        } else {
            ESP_LOGE(TAG, "Failed to register peer");
        }
    }
    digitalWrite(LED::LED_PIN, LOW);

    // Sync Time // FIXME! - Load accesses fault - DONE?
    static bool timeSynced = false;
    if (!timeSynced) {
        if (netClock.initTimeSync(WTbNetConfig::LOAD_MAC)) {
            ESP_LOGI(TAG, "Time sync initialized successfully");
            timeSynced = true;
        } else {
            ESP_LOGE(TAG, "Failed to initialize time sync");
        }
    }
    digitalWrite(LED::LED_PIN, HIGH);

    // Print MAC Address // todo - verify
    ESP_LOGI(
        TAG, "MAC Address: %s",
        AdapterWLAN::formatMACAddress(adapterWLAN.getMACAddress()).c_str());
    digitalWrite(LED::LED_PIN, LOW);

    // TODO: Check ESP-NOW impl against last years
    // TODO: Configure response handler, load server

    digitalWrite(LED::LED_PIN, LOW);

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
                ESP_LOGV(
                    TAG,
                    "Created task %s with priority %u and stack size %u bytes",
                    taskDesc.name, taskDesc.priority, taskDesc.stackSize_bytes);
            }
        }
        tasksSetup = true;

        // pitchPIDController.enable(
        //     0.0f, 1500.0f); // todo - just a quick performances test

        ESP_LOGI(TAG, "Setup complete!");
    }
}

/**
 * MARK: Main Tasks
 * @see
 * https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/01-Tasks-and-co-routines/05-Implementing-a-task
 * @see https://forum.arduino.cc/t/non-blocking-delay-actions/1044079
 */

/**
 * @SuppressWarnings("cpp:S5008") // Does not work
 */
__attribute__((noreturn)) void
vTaskUpdateFSM([[maybe_unused]] void *pvParameters) { // NOSONAR
    while (true) {
        // NacelleFSM::UPDATE_RESULT result = nacelleFSM.updateState();
        // if (result == NacelleFSM::UPDATE_RESULT::STATE_CHANGED) {
        //     ESP_LOGI(TAG, "FSM State Changed: %d",
        //              nacelleFSM.getCurrentState());
        // } else if (result == NacelleFSM::UPDATE_RESULT::ERROR) {
        //     ESP_LOGE(TAG, "Error updating FSM state");
        // }
        delay(RUN::TASK_INTERVALS::TI_FSM_mS);
    }
}

/**
 * @SuppressWarnings("cpp:S5008") // Does not work
 */
__attribute__((noreturn)) void
vTaskPollSensors([[maybe_unused]] void *pvParameters) { // NOSONAR
    while (true) {
        delay(RUN::TASK_INTERVALS::TI_POLL_SENSORS_mS);
    }
}

/**
 * @SuppressWarnings("cpp:S5008") // Does not work
 */
__attribute__((noreturn)) void
vTaskPitch([[maybe_unused]] void *pvParameters) { // NOSONAR
    while (true) {
        if (nacelleFSM.getCurrentState() == FSMCommon::States::sStartLoad ||
            nacelleFSM.getCurrentState() == FSMCommon::States::sSRunLoad) {
            // Fine
        } else if (nacelleFSM.getCurrentState() ==
                   FSMCommon::States::sCurtail) {
            // static int i = 0;
            // ESP_LOGI(TAG, "Pitch PID Output: %f",
            //          pitchPIDController.compute(
            //              i)); // todo - just a quick performances test
            // i += 20;
        } else {
            // No updates needed
            ESP_LOGI(TAG, "No pitch updates needed in state %d, supsending",
                     nacelleFSM.getCurrentState());
            // DONE: Consider suspending when not in use
            vTaskSuspend(
                nacelle
                    .mainTaskDescriptions[NacelleContainer::TASK_IDS::TID_PITCH]
                    .pxHandle); // Suspend until reenabled by FSM
        }
        delay(RUN::TASK_INTERVALS::TI_PITCH_mS);
    }
}

// MARK: Network Tasks

/**
 * @SuppressWarnings("cpp:S5008") // Does not work
 */
__attribute__((noreturn)) void
vTaskRecvData([[maybe_unused]] void *pvParameters) { // NOSONAR
    while (true) {
        if (false) {
            delay(RUN::TASK_INTERVALS::TI_RECV_ms);
        } else {
            // Suspend until reenabled from interrupt
            // vTaskSuspend(mainTaskDescriptions[TASK_IDS::TID_RECV].pxHandle);
            delay(RUN::TASK_INTERVALS::TI_RECV_ms); // TODO: Fix polling
                                                    // (suspend ^ blocks setup)
        }
    }
}

/**
 * @brief Task to handle outbound data that has been queued
 * @SuppressWarnings("cpp:S5008") // Does not work
 */
__attribute__((noreturn)) void
vTaskSendData([[maybe_unused]] void *pvParameters) { // NOSONAR
    while (true) {
        if (false) {
            delay(RUN::TASK_INTERVALS::TI_SEND_ms);
        } else {
            // Suspend until reenabled
            // vTaskSuspend(mainTaskDescriptions[TASK_IDS::TID_SEND].pxHandle);
            delay(RUN::TASK_INTERVALS::TI_SEND_ms); // TODO: Fix polling
                                                    // (suspend ^ blocks setup)
        }
    }
}

// MARK: Utility Tasks

/**
 * @SuppressWarnings("cpp:S5008") // Does not work
 */
__attribute__((noreturn)) void
vTaskConfigure([[maybe_unused]] void *pvParameters) { // NOSONAR
    while (true) {
        // setup(); // todo
        delay(RUN::TASK_INTERVALS::TI_CFG_ms);
    }
}

/**
 * @SuppressWarnings("cpp:S5008") // Does not work
 */
__attribute__((noreturn)) void
vTaskTelnet([[maybe_unused]] void *pvParameters) { // NOSONAR
    while (true) {
        // TELNET::loop(); // todo
        delay(RUN::TASK_INTERVALS::TI_TELNET_ms);
    }
}

/**
 * @SuppressWarnings("cpp:S5008") // Does not work
 */
__attribute__((noreturn)) void
vTaskOTA([[maybe_unused]] void *pvParameters) { // NOSONAR
    while (true) {
        delay(RUN::TASK_INTERVALS::TI_OTA_ms);
    }
}

// MARK: Status Tasks

/**
 * @SuppressWarnings("cpp:S5008") // Does not work
 */
__attribute__((noreturn)) void
vTaskStatusLED([[maybe_unused]] void *pvParameters) { // NOSONAR
    while (true) {
        ESP_LOGV(TAG, "vTSL");
        digitalWrite(LED::LED_PIN, HIGH);
        delay(LED::BLINK_ON_MILLIS);
        digitalWrite(LED::LED_PIN, LOW);
        delay(LED::BLINK_OFF_MILLIS);
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
__attribute__((noreturn)) void
vTaskLogData([[maybe_unused]] void *pvParameters) { // NOSONAR
    /**
     * @See
     * https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32c5/api-reference/peripherals/temp_sensor.html
     * TODO: The temp. sensor may use more power
     * TODO: Temp interput/ callback/ hardware monitoring
     */
    temperature_sensor_handle_t tempSensHandle = NULL;
    temperature_sensor_config_t tempSensConfig =
        TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 100);
    ESP_ERROR_CHECK(
        temperature_sensor_install(&tempSensConfig, &tempSensHandle));

    while (true) {
        // if (!Serial.isConnected()) {
        //     delay(LOG_INTERVAL_MS);
        //     continue;
        // }

        ESP_LOGD(TAG, "Logging Data:");

        ESP_LOGI(TAG, "Num tasks reported by FreeRTOS: %u",
                 uxTaskGetNumberOfTasks());

        constexpr uint_fast8_t REC_BYTES_PER_TASK = 40;
        constexpr uint_fast8_t NUM_ESP_TASKS = 8;
        constexpr uint_fast16_t STATS_BUFFER_SIZE =
            REC_BYTES_PER_TASK * (NUM_MAIN_TASKS + NUM_ESP_TASKS);
        // char statsBuffer[STATS_BUFFER_SIZE] = {'\0'}; // Frowned on by sonar
        // lint:
        etl::string<STATS_BUFFER_SIZE> statsBuffer = {'\0'};
        if (uxTaskGetNumberOfTasks() > NUM_MAIN_TASKS + NUM_ESP_TASKS) {
            ESP_LOGE(
                TAG,
                "Number of tasks (%d) exceeds expected max (%d), skipping to "
                "prevent memory corruption",
                uxTaskGetNumberOfTasks(), NUM_MAIN_TASKS + NUM_ESP_TASKS);
        } else {
            // TODO: Not recommended in production
            vTaskGetRunTimeStats(statsBuffer.data());
            statsBuffer[STATS_BUFFER_SIZE - 1] =
                '\0'; // hard cap, avoid over-read
            statsBuffer.trim_to_terminator();
            // uxTaskGetSystemState();
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
                                            // info for uninitialized tasks, so
                                            // check first
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
        auto tempTrunc_C = (int32_t)tempSens_out;
        constexpr int32_t MAX_EXT_TEMP = 105;
        constexpr int32_t MIN_EXT_TEMP = -40;
        if (tempTrunc_C > MAX_EXT_TEMP || tempTrunc_C < MIN_EXT_TEMP) {
            ESP_LOGE(TAG, "Temperature out of bounds: %d dC", tempTrunc_C);
        } else {
            ESP_LOGI(TAG, "Temperature: %d dC", tempTrunc_C);
        }
        // Disable the temperature sensor if it is not needed and save the power
        ESP_ERROR_CHECK(temperature_sensor_disable(tempSensHandle));
        delay(LOG_ITEM_INTERVAL_MS);

        // esp_wifi_get_bandwidth
        // esp_wifi_sta_get_rssi
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
