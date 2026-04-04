#pragma once

#include "2026Core/FSMStates.hpp"
#include "NacelleConfig.hpp"
#include "NacelleContainer.hpp"
#include <atomic>
#include <cstdint>

/**
 * @brief Class to manage the finite state machine for the nacelle
 */
class NacelleFSM {
  public:
    /**
     * @brief Construct a new Nacelle FSM object
     * @param nacelle The NacelleContainer object that tracks the overall state
     * of the nacelle
     */
    NacelleFSM(NacelleContainer nacelle) : nacelle(nacelle) {
        if (!currentState.is_lock_free()) {
            ESP_LOGE(TAG,
                     "Atomic operations on uint_fast32_t are not lock-free on "
                     "this platform.");
        }
    }
    ~NacelleFSM() = default;

    /**
     * @brief Get the current state of the FSM
     * @return The current state
     */
    FSMCommon::States getCurrentState() const { return currentState; }

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
        // Check safety task
        if (currentState != FSMCommon::sESTOP && nacelle.getSafetyFlag()) {
            currentState = FSMCommon::sESTOP;
            vTaskSuspend(
                nacelle.mainTaskDescriptions[NacelleContainer::TID_PITCH]
                    .pxHandle);
            nacelle.pitchActuator.writePosMicros(
                PITCHING::BLADE_SERVO_STOP_uS); // Feather
            return UPDATE_RESULT::STATE_CHANGED;
        } else if (currentState == FSMCommon::sESTOP &&
                   nacelle.getSafetyFlag()) {
            // Nothing to do
            return UPDATE_RESULT::NO_CHANGE;
        } // else: ~safetyTask

        // Check reset task
        if (currentState != FSMCommon::sRST && !nacelle.isPowerPositive()) {
            currentState = FSMCommon::sRST;
            vTaskSuspend(
                nacelle.mainTaskDescriptions[NacelleContainer::TID_PITCH]
                    .pxHandle);
            nacelle.pitchActuator.writePosMicros(
                PITCHING::BLADE_SERVO_STARTUP_uS); // cut in
            return UPDATE_RESULT::STATE_CHANGED;
        } else if (currentState == FSMCommon::sRST &&
                   !nacelle.isPowerPositive()) {
            // Nothing to do
            return UPDATE_RESULT::NO_CHANGE;
        } // else: producingPositivePower

        // Other transitions
        if (currentState == FSMCommon::sRST) {
            currentState = FSMCommon::sStartLoad;
            vTaskResume(
                nacelle.mainTaskDescriptions[NacelleContainer::TID_PITCH]
                    .pxHandle);
            // todo: pitch fine
        } else if ((currentState == FSMCommon::sStartLoad &&
                    nacelle.isSteadyRPM()) ||
                   (currentState == FSMCommon::sCurtail &&
                    !nacelle.isTargetRPMExceeded())) {
            currentState = FSMCommon::sSRunLoad;
            // Pitch is already set to fine
            // todo signal load
            return UPDATE_RESULT::STATE_CHANGED;
        } else if (currentState == FSMCommon::sSRunLoad &&
                   nacelle.isTargetRPMExceeded()) {
            currentState = FSMCommon::sCurtail;
            // todo pitch PI
            // todo signal load
            return UPDATE_RESULT::STATE_CHANGED;
        } else {
            return UPDATE_RESULT::NO_CHANGE;
        }

        return UPDATE_RESULT::ERROR;
    }

  private:
    NacelleContainer nacelle;

    /**
     * @see https://stackoverflow.com/a/49915536
     */
    static_assert(
        (__cplusplus >= 201703L),
        "C++17 or higher is required for std::atomic is_always_lock_free");

    /**
     * @see https://www.reddit.com/r/embedded/comments/zn23of/comment/j0fav6o/
     * @see
     * https://stackoverflow.com/questions/63471387/should-volatile-still-be-used-for-sharing-data-with-isrs-in-modern-c
     * @see https://en.cppreference.com/w/c/language/atomic.html
     * @see https://en.cppreference.com/w/cpp/atomic/atomic.html
     * @see https://stackoverflow.com/a/16783513
     */
    // static_assert(std::atomic<uint_fast8_t>::is_always_lock_free,
    //               "Atomic operations on uint_fast8_t are not lock-free on "
    //               "this platform.");

    /**
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
     * @see
     * https://stackoverflow.com/questions/21756457/how-can-i-create-an-atomic-enum-in-c
     * @see
     * https://stackoverflow.com/questions/31978324/what-exactly-is-stdatomic
     * @see https://en.cppreference.com/w/cpp/atomic/atomic.html
     */
    std::atomic<FSMCommon::States> currentState = FSMCommon::sINIT;
};
