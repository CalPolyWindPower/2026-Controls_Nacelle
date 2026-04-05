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

    /**
     * @brief Construct a new Nacelle FSM object
     * @param nacelle The NacelleContainer object that tracks the overall state
     * of the nacelle
     */
    NacelleFSM(NacelleContainer nacelle) : nacelle(nacelle) {
        if (!currentState.is_lock_free()) {
            ESP_LOGE(TAG,
                     "Atomic operations on uint_fast8_t are not lock-free on "
                     "this platform.");
        }
        UPDATE_RESULT result = updateState();
        if (result == UPDATE_RESULT::ERROR) {
            ESP_LOGE(TAG, "Error during FSM init., %d", (uint_fast8_t)result);
        } else if (result == UPDATE_RESULT::STATE_CHANGED) {
            ESP_LOGI(TAG, "Initialized FSM to state %d",
                     (uint_fast8_t)currentState.load());
        } else if (result == UPDATE_RESULT::NO_CHANGE) {
            ESP_LOGE(TAG, "Failed to enter a valid state");
        } else {
            ESP_LOGE(TAG, "Unknown FSM init. result: %d", (uint_fast8_t)result);
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
        ERROR = (uint_fast8_t)(uint8_t)-1
    };

    /**
     * @brief Check the inputs and update the FSM state accordingly
     * @return The result of the update, indicating if the state changed or if
     * an error occurred
     */
    UPDATE_RESULT updateState() {
        // Check safety task / E-Stop conditions
        if ((currentState != FSMCommon::States::sESTOP) &&
            nacelle.getSafetyFlag()) {
            // * -> sESTOP
            currentState = FSMCommon::States::sESTOP;

            // Blade pitch -> feather
            vTaskSuspend(
                nacelle.mainTaskDescriptions[NacelleContainer::TID_PITCH]
                    .pxHandle);
            nacelle.pitchActuator.writePosMicros(PITCHING::POS_STOP_uS);

            return UPDATE_RESULT::STATE_CHANGED;
        } else if ((currentState == FSMCommon::States::sESTOP) &&
                   nacelle.getSafetyFlag()) {
            // Nothing to do
            return UPDATE_RESULT::NO_CHANGE;
        } // else: ~safetyTask

        // Check reset conditions
        if ((currentState != FSMCommon::States::sRST) &&
            !nacelle.isPowerPositive()) {
            // * -> sRST
            currentState = FSMCommon::States::sRST;

            // Blade pitch -> cut in
            vTaskSuspend(
                nacelle.mainTaskDescriptions[NacelleContainer::TID_PITCH]
                    .pxHandle);
            nacelle.pitchActuator.writePosMicros(PITCHING::POS_STARTUP_uS);

            return UPDATE_RESULT::STATE_CHANGED;
        } else if ((currentState == FSMCommon::States::sRST) &&
                   !nacelle.isPowerPositive()) {
            // Nothing to do
            return UPDATE_RESULT::NO_CHANGE;
        } // else: producingPositivePower

        // Check other transition conditions
        if (currentState == FSMCommon::States::sRST) {
            // sRST -> sStartLoad
            currentState = FSMCommon::States::sStartLoad;

            // Pitch -> Adjust (fine) // TODO: Check on this
            vTaskResume(
                nacelle.mainTaskDescriptions[NacelleContainer::TID_PITCH]
                    .pxHandle);

            return UPDATE_RESULT::STATE_CHANGED;
        } else if ((currentState == FSMCommon::States::sStartLoad) &&
                   nacelle.isSteadyRPM()) {
            // sStartLoad -> sSRunLoad
            currentState = FSMCommon::States::sSRunLoad;

            // Pitch is already set to adjust (fine)
            // TODO: signal load

            return UPDATE_RESULT::STATE_CHANGED;
        } else if ((currentState == FSMCommon::States::sSRunLoad) &&
                   nacelle.isTargetRPMExceeded()) {
            // sSRunLoad -> sCurtail
            currentState = FSMCommon::States::sCurtail;

            // Pitch is already set to adjust (fine), which will detect the new
            // state (PI)
            // TODO: signal load

            return UPDATE_RESULT::STATE_CHANGED;
        } else if ((currentState == FSMCommon::States::sCurtail) &&
                   !nacelle.isTargetRPMExceeded()) {
            // sCurtail -> sSRunLoad
            currentState = FSMCommon::States::sSRunLoad;

            // Pitch is already set to adjust (PI), which will detect the new
            // state (fine)
            // TODO: signal load

            return UPDATE_RESULT::STATE_CHANGED;
        } else {
            return UPDATE_RESULT::NO_CHANGE;
        }

        return UPDATE_RESULT::ERROR;
    }

  private: // MARK: Private
    NacelleContainer nacelle;

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
