#pragma once

// Standard Library Includes
#include <atomic>
#include <cstdint>

// Project Includes
#include "2026Core/FSMStates.hpp"
#include "NacelleConfig.hpp"
#include "NacelleContainer.hpp"

/**
 * @brief Class to manage the finite state machine for the nacelle
 */
class NacelleFSM {
  public: // MARK: Public
    static constexpr const char *TAG = "NFSM";
    uint32_t targetPower_mW = 40000; // mW
    constexpr static int_fast16_t CURTAIL_EXIT_RPM_TOLERANCE = 400; // RPM tolerance for exiting curtailment, prevents oscillation

    /**
     * @brief Construct a new Nacelle FSM object
     * @param nacelle The NacelleContainer object that tracks the overall state
     * of the nacelle
     */
    NacelleFSM(NacelleContainer &nacelle) : nacelle(nacelle) {
        if (!currentState.is_lock_free()) {
            ESP_LOGE(TAG,
                     "Atomic operations on uint_fast8_t are not lock-free on "
                     "this platform.");
        }
    }
    ~NacelleFSM() = default;

    // MARK: Getters
    /**
     * @brief Get the current state of the FSM
     * @return The current state
     */
    inline FSMCommon::States getCurrentState() const { return currentState; }

    // MARK: State Logic
    /**
     * @brief result of an FSM update/ input check
     */
    enum class UPDATE_RESULT : uint_fast8_t {
        NO_CHANGE = 0,
        STATE_CHANGED = 1,
        /**
         * @details Trim -1 to an 8-bit unsigned integer(255), unsigned extend
         * to uint_fast8_t
         */
        ERROR = static_cast<uint_fast8_t>(static_cast<uint8_t>(-1))
    };

    /**
     * @brief Check the inputs and update the FSM state accordingly
     * @return The result of the update, indicating if the state changed or if
     * an error occurred
     */
    UPDATE_RESULT updateState() {
        ESP_LOGV(TAG, "Updating FSM from state: %d", currentState.load());
        // Check safety task / E-Stop conditions
        if ((currentState != FSMCommon::States::sESTOP) &&
            (nacelle.getSafetyFlag() != ESTOP_TYPE_FAST::NONE)) {
            // * -> sESTOP
            ESP_LOGD(TAG, "* -> sESTOP");
            currentState = FSMCommon::States::sESTOP;

            // Blade pitch -> feather
            vTaskSuspend(
                nacelle.mainTaskDescriptions[NacelleContainer::TID_PITCH]
                    .pxHandle);
            nacelle.pitchPIDController.disable();
            nacelle.pitchActuator.writePosMicros(PITCHING::POS_STOP_uS);

            return UPDATE_RESULT::STATE_CHANGED;
        } else if ((currentState == FSMCommon::States::sESTOP) &&
                   (nacelle.getSafetyFlag() != ESTOP_TYPE_FAST::NONE)) {
            // sESTOP -> sESTOP: Nothing to do
            ESP_LOGV(TAG, "Remaining in sESTOP");
            return UPDATE_RESULT::NO_CHANGE;
        } else {
            // else: ~safetyTask
        }

        // Check reset conditions
        ESP_LOGV(TAG, "Checking reset conditions");
        if ((currentState != FSMCommon::States::sRST) &&
            (nacelle.currentRPM < 200)) { // todo isPowerPositive didn't work
            // * -> sRST
            ESP_LOGD(TAG, "* -> sRST");
            currentState = FSMCommon::States::sRST;

            // Blade pitch -> cut in
            ESP_LOGD(TAG, "Suspending pitch task, %p",
                     nacelle.mainTaskDescriptions[NacelleContainer::TID_PITCH]
                         .pxHandle);
            vTaskSuspend(
                nacelle.mainTaskDescriptions[NacelleContainer::TID_PITCH]
                    .pxHandle);
            ESP_LOGD(TAG, "Disabling pitch PID");
            nacelle.pitchPIDController.disable();
            ESP_LOGD(TAG, "Writing startup pitch position: %u us",
                     PITCHING::POS_STARTUP_uS);
            nacelle.pitchActuator.writePosMicros(PITCHING::POS_STARTUP_uS);

            return UPDATE_RESULT::STATE_CHANGED;
        } else if ((currentState == FSMCommon::States::sRST) &&
                   (nacelle.currentRPM <
                    200)) { // todo isPowerPositive didn't work
            // sRST -> sRST: Nothing to do
            // nacelle.pitchActuator.writePosMicros(
            //     PITCHING::POS_STARTUP_uS); // todo test
            //ESP_LOGI(TAG, "power : %u mW", static_cast<int>(nacelle.power_mW));
            return UPDATE_RESULT::NO_CHANGE;
        } else {
            // else: producingPositivePower or maybe just still starting up
        }

        // Check other transition conditions
        // ESP_LOGD(TAG, "Checking other transition conditions");
        if ((currentState == FSMCommon::States::sRST) &&
            (nacelle.currentRPM > ENCODER::START_RUN_2_RPM)) {
            // sRST -> sStartRun
            currentState = FSMCommon::States::sStartRun;

            // Pitch -> Adjust (fine) // TODO: Check on this
            // Pitch task already disabled
            vTaskResume(
                 nacelle.mainTaskDescriptions[NacelleContainer::TID_PITCH]
                     .pxHandle);
            // PID is already disabled
            //nacelle.pitchActuator.writePosMicros(PITCHING::POS_RUN_uS);
            return UPDATE_RESULT::STATE_CHANGED;
        } else if ((currentState == FSMCommon::States::sStartRun) &&
                   (nacelle.isSteadyRPM())) { // todo
            // sStartRun -> sRunLoad
            // Note: The producing positive power condition is handled by the
            // reset logic
            currentState = FSMCommon::States::sRunLoad;

            // Pitch is already set to adjust (fine)
            // TODO: signal load (set steadyRPM)
            // Load will be sent RPM info elsewhere

            return UPDATE_RESULT::STATE_CHANGED;
        } else if ((currentState == FSMCommon::States::sRunLoad) &&
                   nacelle.isTargetPowerExceeded(targetPower_mW)) { 
            // sRunLoad -> sCurtail
            currentState = FSMCommon::States::sCurtail;
            
            vTaskResume(
                 nacelle.mainTaskDescriptions[NacelleContainer::TID_PITCH]
                     .pxHandle);
            // Pitch is already set to adjust (fine), which will detect the new
            // state (PI)
            nacelle.pitchPIDController.enable(
                nacelle.currentRPM,
                PITCHING::POS_RUN_uS); // todo input & output
            // DONE: signal load (set targetRPMExceeded)
            // Load will be sent RPM info elsewhere
        

            return UPDATE_RESULT::STATE_CHANGED;
        } else if ((currentState == FSMCommon::States::sCurtail) &&
                   nacelle.isTargetRPMExceededHysteresis()) {

            constexpr float POWER_TOLERANCE = 0.05f;
            constexpr int_fast16_t RPM_STEP = 40;

            float powerError = nacelle.power_mW - targetPower_mW; // only checks once, noise could mess this up

            if ((powerError) >=  POWER_TOLERANCE * targetPower_mW) {
                nacelle.pitchPIDController.setTarget(ENCODER::TARGET_RPM - RPM_STEP); // todo adjust decrement amount
                ENCODER::TARGET_RPM = ENCODER::TARGET_RPM - RPM_STEP; // todo adjust decrement amount
                ESP_LOGI(TAG, "Decreasing target RPM to %u due to overpower condition", ENCODER::TARGET_RPM);
            }
            else if (powerError <= (-1) * POWER_TOLERANCE * targetPower_mW) {
                nacelle.pitchPIDController.setTarget(ENCODER::TARGET_RPM + RPM_STEP); // todo adjust increment amount
                ENCODER::TARGET_RPM = ENCODER::TARGET_RPM + RPM_STEP; // todo adjust increment amount
                ESP_LOGI(TAG, "Increasing target RPM to %u due to underpower condition", ENCODER::TARGET_RPM);
            }

            else {
                nacelle.pitchPIDController.disable();
                ESP_LOGI(TAG, "Power within acceptable range, PID disabled");
            }

            // sCurtail -> sRunLoad
            //currentState = FSMCommon::States::sRunLoad;

            // Pitch is already set to adjust (PI), which will detect the new
            // state (fine)

            //nacelle.pitchPIDController.disable();

            // DONE: signal load (unset targetRPMExceeded)
            // Load will be sent RPM info elsewhere
            return UPDATE_RESULT::NO_CHANGE;
            //return UPDATE_RESULT::STATE_CHANGED;
        } else if ((currentState == FSMCommon::States::sCurtail) &&
                   (nacelle.currentRPM < ENCODER::TARGET_RPM - CURTAIL_EXIT_RPM_TOLERANCE)) {
            currentState = FSMCommon::States::sRunLoad;
            ESP_LOGI(TAG, "Current RPM below target RPM, transitioning back to sRunLoad");
            return UPDATE_RESULT::STATE_CHANGED;
        }
        
        else {
            return UPDATE_RESULT::NO_CHANGE;
        }

        ESP_LOGE(TAG, "Reached end of updateState");
        return UPDATE_RESULT::ERROR;
    }

    void setTargetPower(float targetPower_mW) {
        this->targetPower_mW = targetPower_mW;
    }

  private:                     // MARK: Private
    NacelleContainer &nacelle; // DONE: switch to by reference

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
    static_assert(sizeof(int) == sizeof(uint_fast8_t),
                  "Atomic Lock-free check issue");
#if (ATOMIC_INT_LOCK_FREE == 0)
#    error "Atomic operations on int are not lock-free on this platform."
#elif (ATOMIC_INT_LOCK_FREE == 1)
#    warning                                                                   \
        "Atomic operations on int are only sometimes lock-free on this platform."
#endif

    /**
     * @brief Check if std::atomic<uint_fast8_t> is an acceptable (lock free)
     * solution for shared variables
     * @see https://www.reddit.com/r/embedded/comments/zn23of/comment/j0fav6o/
     * @see
     * https://stackoverflow.com/questions/63471387/should-volatile-still-be-used-for-sharing-data-with-isrs-in-modern-c
     * @see https://en.cppreference.com/w/c/language/atomic.html
     * @see https://en.cppreference.com/w/cpp/atomic/atomic.html
     * @see https://stackoverflow.com/a/16783513
     */
    static_assert(std::atomic<uint_fast8_t>::is_always_lock_free,
                  "Atomic operations on uint_fast8_t are not lock-free on "
                  "this platform.");

    /**
     * @details Store the FSM state as an atomic variable such that it can be
     * safely accessed from multiple tasks
     * @see
     * https://stackoverflow.com/questions/21756457/how-can-i-create-an-atomic-enum-in-c
     * @see
     * https://stackoverflow.com/questions/31978324/what-exactly-is-stdatomic
     * @see https://en.cppreference.com/w/cpp/atomic/atomic.html
     */
    std::atomic<FSMCommon::States> currentState = FSMCommon::States::sINIT;
};
