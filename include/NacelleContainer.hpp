#pragma once

#include <cstdint>

// Third Party Libraries
#include <ActuonixL12.hpp>
#include <etl/format_spec.h>
#include <etl/string.h>
#include <etl/to_string.h>
#include <cstdlib>  // or <cmath>

// Custom Includes
#include "NacelleComms.hpp"
#include "NacelleTasks.hpp"
#include "PID.hpp"


/**
 * @brief Class to manage the container for nacelle data
 */
class NacelleContainer {
  public:
    static constexpr const char *TAG = "NC";

    // constexpr uint_fast8_t NUM_MAIN_TASKS = 1;
    // Arduino Loop has priority 1
    // TODO: Note: Task priority must be < 25
    etl::array<TaskInfo, NUM_MAIN_TASKS> mainTaskDescriptions = {
        TaskInfo{vTaskUpdateFSM, "FSM", 2048, nullptr, 24, nullptr, 0,
                 false}, // 0
        TaskInfo{vTaskPollSensors, "Poll", 2048, nullptr, 15, nullptr, 0,
                 false},                                                    // 1
        TaskInfo{vTaskPitch, "Ptch", 4096, nullptr, 15, nullptr, 0, false}, // 2
        TaskInfo{vTaskRecvData, "Recv", 2048, nullptr, 10, nullptr, 0,
                 false}, // 3

        TaskInfo{vTaskSendData, "Send", 2048, nullptr, 10, nullptr, 0,
                 false}, // 4
        // Got an interminnat stack protection fault at 512 bytes, trying 1024
        // bytes
        TaskInfo{vTaskConfigure, "Cfg", 1024, nullptr, 5, nullptr, 0,
                 false}, // 5
        TaskInfo{vTaskStatusLED, "LED", 512, nullptr, 2, nullptr, 0,
                 false},                                                   // 6
        TaskInfo{vTaskLogData, "Log", 4096, nullptr, 1, nullptr, 0, false} // 7
    };

    etl::array<TaskInfo, NUM_OPTIONAL_TASKS> optionalTaskDescriptions = {
        TaskInfo{vTaskTelnet, "Telnet", 4096, nullptr, 1, nullptr, 0,
                 false},                                                // 0
        TaskInfo{vTaskOTA, "OTA", 4096, nullptr, 1, nullptr, 0, false}, // 1
    };

    /**
     * @SupressWarnings("cpp:S3642") // Does not work
     */
    enum MAIN_TASK_IDS : uint_fast8_t { // NOSONAR
        TID_FSM = 0,
        TID_POLL,
        TID_PITCH,
        TID_RECV,
        TID_SEND,
        TID_CFG,
        TID_LED,
        TID_LOG
    };

    enum OPT_TASK_IDS : uint_fast8_t { // NOSONAR
        TID_TELNET = 0,
        TID_OTA
    };

    ActuonixL12 &pitchActuator;
    PID &pitchPIDController;
    NacelleComms &nacelleComms;

    /**
     * @brief Check for C++17 support, which allows us to verify if std::atomic
     * is a acceptable (lock free) solution for shared variables
     * @see https://stackoverflow.com/a/49915536
     */
    static_assert(
        (__cplusplus >= 201703L),
        "C++17 or higher is required for std::atomic is_always_lock_free");

    /**
     * @brief Do some basic checks regarding std::atomic and data types
     * From C++14.2.0 atomic.h:
     * Check Lock-free property.
     *
     * 0 indicates that the types are never lock-free.
     * 1 indicates that the types are sometimes lock-free.
     * 2 indicates that the types are always lock-free.
     */
    static_assert(sizeof(int) == sizeof(int_fast16_t),
                  "Atomic Lock-free check issue");
// Can't use floats atomically
// #if (ATOMIC_FLOAT_LOCK_FREE == 0)
// #    error "Atomic operations on float are not lock-free on this platform."
// #elif (ATOMIC_FLOAT_LOCK_FREE == 1)
// #    warning \
    //         "Atomic operations on float are only sometimes lock-free on this
//         platform."
// #endif
#if (ATOMIC_INT_LOCK_FREE == 0)
#    error "Atomic operations on int are not lock-free on this platform."
#elif (ATOMIC_INT_LOCK_FREE == 1)
#    warning                                                                   \
        "Atomic operations on int are only sometimes lock-free on this platform."
#endif

    /**
     * @brief Check if std::atomic<int_fast16_t> is an acceptable (lock free)
     * solution for shared variables
     * @see https://www.reddit.com/r/embedded/comments/zn23of/comment/j0fav6o/
     * @see
     * https://stackoverflow.com/questions/63471387/should-volatile-still-be-used-for-sharing-data-with-isrs-in-modern-c
     * @see https://en.cppreference.com/w/c/language/atomic.html
     * @see https://en.cppreference.com/w/cpp/atomic/atomic.html
     * @see https://stackoverflow.com/a/16783513
     */
    static_assert(std::atomic<int_fast16_t>::is_always_lock_free,
                  "Atomic operations on int_fast16_t are not lock-free on "
                  "this platform.");
    std::atomic<int_fast16_t> currentRPM = 0;
    std::atomic<int_fast16_t> angularAccel_RPMPS = 0;
    std::atomic<int16_t> d_mVPS = 0;
    std::atomic<int16_t> current_mA = 0;
    std::atomic<int16_t> dIPS = 0;

    NacelleContainer(ActuonixL12 &pitchActuator, PID &pitchPIDController,
                     NacelleComms &nacelleComms)
        : pitchActuator(pitchActuator), pitchPIDController(pitchPIDController),
          nacelleComms(nacelleComms) {}
    ~NacelleContainer() = default;

    inline ESTOP_TYPE_FAST getSafetyFlag() const { return safetyFlag; }
    inline bool isPowerPositive() const {
        return true;
        // return powerPositive;
    }
    /**
     * @deprecated Apparently we don't need to check this
     */
    inline bool isSteadyRPM() const {
        return true;
        // return angularAccel_RPMPS < 20;
    } // todo
    inline bool isTargetRPMExceeded() const {
        return (currentRPM > ENCODER::TARGET_RPM);
    }


    // checks if the rpm has been in a steady state (within tolerance) for a certain duration
    bool isTargetRPMExceededHysteresis()  {
        const TickType_t now = xTaskGetTickCount();
        if (std::abs(static_cast<int32_t>(currentRPM) -
             static_cast<int32_t>(ENCODER::TARGET_RPM)) < steadyRpmTolerance) {
            if (initialTime != 0) {
                if ((now - initialTime) >= pdMS_TO_TICKS(MIN_STEADY_STATE_DURATION_MS)) {
                    return true;
                }
            }
            else {
                initialTime = xTaskGetTickCount();
            }
        }
        else {
            initialTime = 0;
        }
        return false;
    }

    static constexpr uint_fast8_t LOG_STRING_SIZE =
        3 + 7 + 5 + 10 + 5 + ((6 + 1) * 3) + 7 + 1 +
        1; // TODO - improve this and null terminator may not be needed
    /**
     * @brief Get at string that describes the current state of the PID instance
     * @returns the current state of the PID instance as a string
     */
    etl::string<LOG_STRING_SIZE> getLogString() {
        etl::string<LOG_STRING_SIZE> logString(TAG); // 3 chars
        (void)logString.append(": RPM: ");           // 7 chars

        etl::format_spec decFormatA;
        (void)decFormatA.width(5).fill('0'); // [5 chars]
        /**
         * @details I don't think we need strong guarantees on logging data
         * @see
         * https://stackoverflow.com/questions/12346487/what-do-each-memory-order-mean
         * @see https://en.cppreference.com/cpp/atomic/memory_order
         */
        etl::to_string(currentRPM.load(std::memory_order::relaxed), logString,
                       decFormatA, true);     // 5 chars
        (void)logString.append(", dRPM/s: "); // 10 chars

        etl::to_string(angularAccel_RPMPS.load(std::memory_order::relaxed),
                       logString, decFormatA,
                       true); // 5 chars

        (void)logString.append(", SF: "); // 6 chars

        etl::format_spec decFormatB;
        decFormatB.width(1).fill('0'); // [1 chars]
        etl::to_string(static_cast<uint_fast8_t>(getSafetyFlag()), logString,
                       decFormatB, true); // 1 char

        etl::format_spec boolFormatA;
        boolFormatA.binary().width(1).fill('0'); // [1 char]
        (void)logString.append(", PP: ");        // 6 chars, 1 char \/
        etl::to_string(isPowerPositive(), logString, boolFormatA, true);
        (void)logString.append(", SR: ");                            // 6 chars
        etl::to_string(isSteadyRPM(), logString, boolFormatA, true); // 1 char
        (void)logString.append(", TRE: "); // 7 chars, 1 char \/
        etl::to_string(isTargetRPMExceeded(), logString, boolFormatA, true);

        return logString;
    }

    void setSafetyFlag(ESTOP_TYPE_FAST safetyFlag) {
        // Debug
        if (!enableSafetyFlag) {
            // Don't use safety flag
            this->safetyFlag = ESTOP_TYPE_FAST::NONE; // todo ?
            return;
        }

        // E-stop button
        if (safetyFlag == ESTOP_TYPE_FAST::BUTTON) {
            this->safetyFlag = ESTOP_TYPE_FAST::BUTTON;
            return;
        }

        // Load disconnect - inferred from low current
        if (safetyFlag == ESTOP_TYPE_FAST::LOAD_DISCONNECT_I) {
            this->safetyFlag = ESTOP_TYPE_FAST::LOAD_DISCONNECT_I;
            return;
        }

        // todo: Fix RPM/s calc
        // Load disconnect - inferred from high acceleration
        // static uint_fast16_t loadDisconnectCount = 0;
        // if (currentRPM > (ENCODER::TARGET_RPM * 0.22f) && // 500
        //     angularAccel_RPMPS > 500) {
        //     // A high acceleration indicates a load disconnect
        //     loadDisconnectCount++;
        //     if (loadDisconnectCount > 50) {
        //         this->safetyFlag = ESTOP_TYPE_FAST::LOAD_DISCONNECT_E;
        //     }
        //     return;
        // } else if (this->safetyFlag == ESTOP_TYPE_FAST::LOAD_DISCONNECT_E &&
        //            currentRPM < (ENCODER::TARGET_RPM * 0.05f)) {
        //     loadDisconnectCount = 0;
        //     this->safetyFlag = ESTOP_TYPE_FAST::NONE;
        //     return;
        // }
        // todo: check current limit, hysteresis

        // Unset E-stop button
        if (this->safetyFlag == ESTOP_TYPE_FAST::BUTTON &&
            safetyFlag == ESTOP_TYPE_FAST::NONE) {
            this->safetyFlag = ESTOP_TYPE_FAST::NONE;
            return;
        }

        if (this->safetyFlag == ESTOP_TYPE_FAST::LOAD_DISCONNECT_I &&
            safetyFlag == ESTOP_TYPE_FAST::NONE) {
            this->safetyFlag = ESTOP_TYPE_FAST::NONE;
            return;
        }

        if (this->safetyFlag == ESTOP_TYPE_FAST::LOAD_DISCONNECT_E &&
            safetyFlag == ESTOP_TYPE_FAST::NONE) {
            this->safetyFlag = ESTOP_TYPE_FAST::NONE;
            return;
        }

        // Else: No estop needed
        // ESP_LOGE(TAG, "Unexpectedly reached end of SF decision tree");
    }
    inline void updatePowerPositive(bool powerPositive) {
        this->powerPositive = powerPositive;
    }

    inline void setEnableSafetyFlag(bool enable) {
        this->enableSafetyFlag = enable;
    }

  private:
    std::atomic<ESTOP_TYPE_FAST> safetyFlag = ESTOP_TYPE_FAST::NONE;
    bool powerPositive = false;
    bool enableSafetyFlag = true;
    TickType_t initialTime = 0;
    constexpr static uint32_t MIN_STEADY_STATE_DURATION_MS = 5000; // 5 seconds
    constexpr static uint8_t steadyRpmTolerance = 20; // RPM tolerance for steady state check
};
