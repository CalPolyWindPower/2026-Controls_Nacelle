#pragma once

#include "2026Core/FSMStates.hpp"
#include "NacelleConfig.hpp"
#include "NacelleContainer.hpp"
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
    NacelleFSM(NacelleContainer nacelle) : nacelle(nacelle) {}
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
    FSMCommon::States currentState = FSMCommon::sINIT;
};
