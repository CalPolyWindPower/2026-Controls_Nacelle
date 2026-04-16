/**
 * Adapted from the Arduino PID Library, Version 1.2.1 by
 * @author Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 * The original code is licensed under the MIT license.
 *
 * @see https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
 * @see
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 */

#ifndef PID_hpp
#define PID_hpp

#include <Arduino.h>
#include <etl/format_spec.h>
#include <etl/string.h>
#include <etl/to_string.h>

class PID {

  public: // MARK: Public
    /* Constants used in some of the functions below */

    enum PIDMODE : uint8_t { MANUAL = 0, AUTOMATIC = 1 };

    enum Direction : uint8_t { DIRECT = 0, REVERSE = 1 };

    enum ProportionalMode : uint8_t {
        ProportionalOnMeas = 0,
        ProportionalOnErr = 1
    };

    static constexpr const char *TAG = "PID";

    /* Constructor */

    /**
     * @brief Constructor
     * @param kP Proportional tuning parameter
     * @param kI Integral tuning parameter
     * @param kD Derivative tuning parameter
     * @param setpoint Setpoint
     * @param proportionalMode Set proportional mode
     * @param minOutput Minimum output value to clamp to
     * @param maxOutput Maximum output value to clamp to
     * @param minSampleTime Minimum sample time in microseconds, or 0 if unused
     * @param dir Set the direction of the controller
     */
    PID(float kP, float kI, float kD, float setpoint,
        ProportionalMode propMode = ProportionalMode::ProportionalOnErr,
        float minOutput = 0, float maxOutput = 255,
        unsigned long minSampleTime = 100, Direction dir = Direction::DIRECT,
        const etl::string<10> n = "PID");

    // MARK: Updaters

    /**
     * @brief Run the PID controller once, should be called regularly
     * Performs the PID calculation.  it should be called every time loop()
     * cycles. ON/OFF and calculation frequency can be se using
     * SetMode and SetSampleTime respectively
     * @param output The computation result
     */
    float compute(float input);

    // MARK: Setters

    /**
     * @brief Sets the setpoint
     * @param target setpoint to set
     */
    void setTarget(float target);

    /**
     * @brief sets PID to either Manual Auto
     * @param mode
     * @param input
     * @param currentOutput
     */
    void setMode(PIDMODE mode, float input, float currentOutput);

    /**
     * @brief sets PID to Auto
     * @param input
     * @param currentOutput
     */
    void enable(float input, float currentOutput);

    /**
     * @brief sets PID to either Manual
     */
    void disable();

    /**
     * @brief clamps the output to a specific range, default is 0-255
     * It's likely the user will want to change this depending on the
     * application
     * @param min minimum output value
     * @param max maximum output value
     */
    bool setOutputLimits(float min = 0, float max = 255);

    /* Available but not commonly used functions */

    /**
     * While most users will set the tunings once in the constructor, this
     * function gives the user the option of changing tunings during runtime for
     * Adaptive control
     * @param kP Proportional tuning parameter
     * @param kI Integral tuning parameter
     * @param kD Derivative tuning parameter
     * @returns true if the tunings were set successfully, false otherwise
     */
    bool setTunings(float kP, float kI, float kD);

    /**
     * While most users will set the tunings once in the constructor, this
     * function gives the user the option of changing tunings during runtime for
     * Adaptive control
     * @param kP Proportional tuning parameter
     * @param kI Integral tuning parameter
     * @param kD Derivative tuning parameter
     * @param propMode Set proportional mode
     * @returns true if the tunings were set successfully, false otherwise
     */
    bool setTunings(float kP, float kI, float kD, ProportionalMode propMode);

    /**
     * Sets the Direction, or "Action" of the controller. DIRECT means the
     * output will increase when error is positive. REVERSE means the opposite.
     * it's very unlikely that this will be needed once it is set in the
     * constructor.
     * @param dir Set the direction of the controller
     * @returns true if the direction was set successfully, false otherwise
     */
    bool setControllerDirection(Direction dir);

    /**
     * sets the frequency, in microseconds, with which the PID calculation is
     * performed.  default is 100
     * @param minSampleTime Minimum sample time in microseconds
     * @returns true if the sample time was set successfully, false otherwise
     */
    bool setSampleTime(unsigned long minTimeMicros);

    // MARK: Getters
    /* Getters / Display / Status functions
    These functions query the pid for internal values.  They were created mainly
    for the pid front-end, where it's important to know what is actually
    inside the PID. */

    /**
     * @brief Get the user supplied Kp value
     * @returns the user supplied Kp value
     */
    float getUserKp() const;

    /**
     * @brief Get the user supplied Ki value
     * @returns the user supplied Ki value
     */
    float getUserKi() const;

    /**
     * @brief Get the user supplied  Kd value
     * @returns the user supplied  Kd value
     */
    float getUserKd() const;

    /**
     * @brief Get the internal supplied Kp value
     * @returns the internal supplied Kp value
     */
    float getInternalKp() const;

    /**
     * @brief Get the internal supplied Ki value
     * @returns the internal supplied Ki value
     */
    float getInternalKi() const;

    /**
     * @brief Get the internal Kd value
     * @returns the internal internal Kd value
     */
    float getInternalKd() const;

    /**
     * @brief Get the enabled state
     * @returns the enabled state
     */
    bool isEnabled() const;

    /**
     * @brief Get the enabled state as PIDMode enum
     * @returns the mode value
     */
    PIDMODE getMode() const;

    /**
     * @brief Get the direction value
     * @returns the direction value
     */
    Direction getDirection() const;

    /**
     * @brief Get PID Proportional Mode
     * @returns the direction value  */
    ProportionalMode getProportionalMode() const;

    /**
     * @brief Get the minimum sample time
     * @returns the minimum sample time
     */
    unsigned long getMinSampleTime() const;

    /**
     * @brief Get the minimum output value
     * @returns the minimum output value
     */
    float getMinOutput() const;

    /**
     * @brief Get the maximum output value
     * @returns the maximum output value
     */
    float getMaxOutput() const;

    /**
     * @brief Get the Setpoint value
     * @returns the Setpoint value
     */
    float getSetpoint() const;

    static constexpr uint_fast8_t LOG_STRING_SIZE =
        3 + 10 + 6 + 8 + 4 + 6 + 4 + 7 + ((6 + 7) * 2) + 1 + 7 + 4 + 5 + 1;
    /**
     * @brief Get at string that describes the current state of the PID instance
     * @returns the current state of the PID instance as a string
     */
    etl::string<LOG_STRING_SIZE> getLogString();

  private: // MARK: Private
    /* Private Helper Functions */

    /**
     * Does all the things that need to happen to ensure a bumpless transfer
     * from manual to automatic mode.
     * @param input
     * @param currentOutput
     */
    void initialize(float input, float currentOutput);

    /* Working Vars */
    const etl::string<10> name;
    unsigned long lastTime;
    float outputSum = 0;
    float lastInput = 0;

    /* Tuning and Mode Configuration */
    float dispKp;     // * (P)roportional tuning Parameter
    float dispKi;     // * (I)ntegral tuning Parameter
    float dispKd;     // * (D)erivative tuning Parameter
    float internalKp; // * (P)roportional tuning Parameter
    float internalKi; // * (I)ntegral tuning Parameter
    float internalKd; // * (D)erivative tuning Parameter
    float setpoint;
    float outMin;
    float outMax;
    unsigned long minSampleTime;
    ProportionalMode proportionalMode;
    Direction controllerDirection;
    bool enabled = false;
};

#endif // PID_hpp
