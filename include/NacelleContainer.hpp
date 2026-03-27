#pragma once

#include <cstdint>

/**
 * @brief Class to manage the container for nacelle data
 */
class NacelleContainer {
  public:
    NacelleContainer() = default;
    ~NacelleContainer() = default;

    bool getSafetyFlag() const { return safetyFlag; }
    bool isPowerPositive() const { return powerPositive; }
    bool isSteadyRPM() const { return false; }         // todo
    bool isTargetRPMExceeded() const { return false; } // todo

    void updateSafetyFlag(bool safetyFlag) { this->safetyFlag = safetyFlag; }
    void updatePowerPositive(bool powerPositive) {
        this->powerPositive = powerPositive;
    }

  private:
    bool safetyFlag = false;
    bool powerPositive = false;
    int_fast16_t currentRPM = 0; // todo
};
