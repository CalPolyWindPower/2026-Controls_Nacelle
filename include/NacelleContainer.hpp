#pragma once

#include <cstdint>

#include <ActuonixL12.hpp>

/**
 * @brief Class to manage the container for nacelle data
 */
class NacelleContainer {
  public:
    ActuonixL12 &pitchActuator;

    NacelleContainer(ActuonixL12 &pitchActuator)
        : pitchActuator(pitchActuator) {}
    ~NacelleContainer() = default;

    inline bool getSafetyFlag() const { return safetyFlag; }
    inline bool isPowerPositive() const { return powerPositive; }
    inline bool isSteadyRPM() const { return false; }         // todo
    inline bool isTargetRPMExceeded() const { return false; } // todo

    inline void updateSafetyFlag(bool safetyFlag) {
        this->safetyFlag = safetyFlag;
    }
    inline void updatePowerPositive(bool powerPositive) {
        this->powerPositive = powerPositive;
    }

  private:
    bool safetyFlag = false;
    bool powerPositive = false;
    int_fast16_t currentRPM = 0; // todo
};
