/**
 * Adapted from the Arduino PID Library, Version 1.2.1 by
 * @author Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 * The original code is licensed under the MIT license.
 *
 * @see https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.cpp
 * @see
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 */

#include <PID.hpp>

// MARK: Constructor

PID::PID(float kP, float kI, float kD, float setpoint,
         ProportionalMode propMode, float minOutput, float maxOutput,
         unsigned long minSampleTime, Direction dir, const etl::string<10> n)
    : setpoint(setpoint), outMin(minOutput), outMax(maxOutput),
      minSampleTime(minSampleTime), controllerDirection(dir), name(n) {

    this->setTunings(kP, kI, kD, propMode);

    this->lastTime = micros() - this->minSampleTime;

    this->enabled = false; // in manual mode by default; call enable() to start!
    this->outputSum = 0;
}

// MARK: Updaters

/**
 * @details This, as they say, is where the magic happens.  this function
 * should be called every time "void loop()" executes.  the function will
 * decide for itself whether a new pid Output needs to be computed.
 * @see
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-derivative-kick/
 * @see
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-reset-windup
 */
float PID::compute(float input) {
    // Check if the PID is enabled (auto mode)
    if (!this->enabled) {
        ESP_LOGW(TAG, "PID not enabled, skipping computation");
        return -1;
    }

    // Time Math
    unsigned long currentTime = micros();
    unsigned long deltaTime = (currentTime - this->lastTime);
    ESP_LOGV(TAG, "deltaTime: %lu", deltaTime);

    // Check if the minimum sample time has not passed
    if (this->minSampleTime != 0 && deltaTime <= this->minSampleTime) {
        ESP_LOGW(TAG, "Sample time too short");
        return -2;
    }

    // Normal Computation

    /* Compute all the working error variables */
    float error = this->setpoint - input;
    ESP_LOGV(TAG, "error: %f", error);

    float deltaInput = (input - this->lastInput);
    this->outputSum += (this->internalKi * (float)deltaTime * error * 1e-6);
    // constexpr float maxSum = 0.1 * 20000 * 500;
    ESP_LOGV(TAG, "outputSum: %f", this->outputSum);

    // [DONE?] time correction

    /*Add Proportional on Measurement, if P_ON_M is specified*/
    if (this->proportionalMode == ProportionalMode::ProportionalOnMeas) {
        this->outputSum -= this->internalKp * deltaInput;
        ESP_LOGV(TAG, "PoM outputSum: %f", this->internalKp * deltaInput);
    }

    /**
     * @brief Constrain the output sum to the specified limits
     * @see
     * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-reset-windup
     */
    if (this->outputSum > this->outMax) {
        ESP_LOGD(TAG, "PID outputSum capped to max");
        this->outputSum = this->outMax;
    } else if (this->outputSum < this->outMin) {
        ESP_LOGD(TAG, "PID outputSum capped to min");
        this->outputSum = this->outMin;
    }

    float output;
    /*Add Proportional on Error, if P_ON_E is specified*/
    if (proportionalMode == ProportionalMode::ProportionalOnErr) {
        output = this->internalKp * error;
        ESP_LOGV(TAG, "PoE output: %f", output);
    } else {
        output = 0;
    }

    /*Compute Rest of PID Output*/
    output += this->outputSum -
              this->internalKd / ((float)deltaTime * 1e-6) * deltaInput;
    ESP_LOGV(TAG, "Fin. Output: %f", output);

    /**
     * @brief  Constrain the output to the specified limits
     * @see
     * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-reset-windup
     */
    if (output > this->outMax) {
        ESP_LOGD(TAG, "PID output capped to max");

        output = this->outMax;
    } else if (output < this->outMin) {
        ESP_LOGD(TAG, "PID output capped to min");
        output = this->outMin;
    }

    /*Remember some variables for next time*/
    this->lastInput = input;
    this->lastTime = currentTime;

    return output;
}

// MARK: Setters
void PID::setTarget(float target) { this->setpoint = target; }

/**
 * @see
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-onoff
 */
void PID::setMode(PIDMODE mode, float input, float currentOutput) {
    if (!this->enabled && mode == PIDMODE::AUTOMATIC) {
        // Enable PID
        this->enabled = true;
        /*we just went from manual to auto*/
        this->initialize(input, currentOutput);
    } else if (this->enabled && mode == PIDMODE::MANUAL) {
        // Disable PID
        this->enabled = false;
    }
}

/**
 * @see
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-onoff
 */
void PID::enable(float input, float currentOutput) {
    if (!(this->enabled)) {
        ESP_LOGI(TAG, "Enabling PID controller...");
        // Enable PID
        this->enabled = true;
        /*we just went from manual to auto*/
        this->initialize(input, currentOutput);
    } else {
        ESP_LOGW(TAG, "PID controller already enabled");
    }
}

/**
 * @see
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-onoff
 */
void PID::disable() {
    if (this->enabled) {
        ESP_LOGI(TAG, "Disabling PID controller...");

        // Disable PID
        this->enabled = false;
    }
}

bool PID::setOutputLimits(float min, float max) {
    // Validate inputs
    if (min >= -max) {
        return false;
    }

    // Set new limits
    this->outMin = min;
    this->outMax = max;

    // Make adjustments
    if (enabled) {
        // todo - is this correct?
        if (this->outputSum > this->outMax)
            this->outputSum = this->outMax;
        else if (this->outputSum < this->outMin)
            this->outputSum = this->outMin;
    }

    return true;
}

// MARK: Other Setters
/* Available but not commonly used functions */

/**
 * @details  This function allows the controller's dynamic performance to be
 * adjusted. it's called automatically from the constructor, but tunings can
 * also be adjusted on the fly during normal operation otherwise
 * @see
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes
 */
bool PID::setTunings(float kP, float kI, float kD) {
    return setTunings(kP, kI, kD, this->proportionalMode);
}

bool PID::setTunings(float kP, float kI, float kD, ProportionalMode propMode) {
    // Validate tunings
    if (kP < 0 || kI < 0 || kD < 0) {
        ESP_LOGE(TAG,
                 "Invalid PID tunings: kP, kI, and kD must be non-negative");
        return false;
    }

    this->proportionalMode = propMode;

    this->dispKp = kP;
    this->dispKi = kI;
    this->dispKd = kD;

    internalKp = kP;
    internalKi = kI;
    internalKd = kD;

    /**
     * @See docs folder for explanation on "if" initializer from SonarLint
     */
    if (float SampleTimeInSec = ((float)this->minSampleTime) / 1000000;
        SampleTimeInSec > 0) {
        ESP_LOGI(TAG, "Adjusting PID values for sample time...");
        internalKi *= SampleTimeInSec;
        internalKd /= SampleTimeInSec;
    }

    if (controllerDirection == Direction::REVERSE) {
        ESP_LOGI(TAG, "Reversing PID values...");
        this->internalKp = (0 - this->internalKp);
        this->internalKi = (0 - this->internalKi);
        this->internalKd = (0 - this->internalKd);
    }

    ESP_LOGI(TAG, "PID tunings set!");
    ESP_LOGI(TAG, "internalKp: %f", this->internalKp);
    ESP_LOGI(TAG, "internalKi: %f", this->internalKi);
    ESP_LOGI(TAG, "internalKd: %f", this->internalKd);

    return true;
}

/**
 * @details The PID will either be connected to a DIRECT acting process
 * (+Output leads to +Input) or a REVERSE acting process(+Output leads to
 * -Input.)  we need to know which one, because otherwise we may increase
 * the output when we should be decreasing.  This is called from the
 * constructor.
 * @see
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction
 */
bool PID::setControllerDirection(Direction dir) {
    if (this->enabled && dir != this->controllerDirection) {
        this->internalKp = (0 - this->internalKp);
        this->internalKi = (0 - this->internalKi);
        this->internalKd = (0 - this->internalKd);
        this->controllerDirection = dir;
        return true;
    } else {
        return false;
    }
}

/**
 * @details sets the period, in Milliseconds, at which the calculation is
 * performed
 * @see
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-sample-time
 */
bool PID::setSampleTime(unsigned long minTimeMicros) {
    if (minTimeMicros <= 0) {
        return false;
    }

    float ratio = (float)minTimeMicros / (float)this->minSampleTime;
    internalKi *= ratio;
    internalKd /= ratio;
    this->minSampleTime = minTimeMicros;
    return true;
}

// MARK: Getters
/* Getters / Display / Status functions
These functions query the pid for internal values.  They were created mainly
for the pid front-end, where it's important to know what is actually
inside the PID. */

float PID::getUserKp() const { return this->dispKp; }

float PID::getUserKi() const { return this->dispKi; }

float PID::getUserKd() const { return this->dispKd; }

float PID::getInternalKp() const { return this->internalKp; }

float PID::getInternalKi() const { return this->internalKi; }

float PID::getInternalKd() const { return this->internalKd; }

bool PID::isEnabled() const { return this->enabled; }

PID::PIDMODE PID::getMode() const {
    if (enabled) {
        return PIDMODE::AUTOMATIC;
    } else {
        return PIDMODE::MANUAL;
    }
}

PID::Direction PID::getDirection() const { return this->controllerDirection; }

/**
 * @see
 * http://brettbeauregard.com/blog/2017/06/proportional-on-measurement-the-code/
 */
PID::ProportionalMode PID::getProportionalMode() const {
    return this->proportionalMode;
}

unsigned long PID::getMinSampleTime() const { return this->minSampleTime; }

float PID::getMinOutput() const { return this->outMin; }

float PID::getMaxOutput() const { return this->outMax; }

float PID::getSetpoint() const { return this->setpoint; }

/**
 * @see https://stackoverflow.com/questions/3350385/how-to-return-an-object-in-c
 */
etl::string<PID::LOG_STRING_SIZE> PID::getLogString() {
    etl::string<LOG_STRING_SIZE> logString(TAG); // 3 chars
    logString.append(": lTus: 0x");              // 10 chars

    etl::format_spec hexFormatA;
    hexFormatA.hex().width(6).fill('0');                   // [6 chars]
    etl::to_string(lastTime, logString, hexFormatA, true); // 6 c
    logString.append(", oS: 0x");                          // 8 chars

    etl::format_spec hexFormatB;
    hexFormatB.hex().width(4).fill('0');                    // [4 chars]
    etl::to_string(outputSum, logString, hexFormatB, true); // 4 c
    logString.append(", lI: ");                             // 6 chars

    etl::format_spec decFormatA;
    decFormatA.width(4).fill('0');                          // [4 chars]
    etl::to_string(lastInput, logString, decFormatA, true); // 4 c
    logString.append(", iKp: ");                            // 7 chars

    etl::format_spec decFormatB;
    decFormatB.width(6).fill('0');                           // [6 chars]
    etl::to_string(internalKp, logString, decFormatB, true); // 6 chars
    logString.append(", iKi: ");                             // 7 chars
    etl::to_string(internalKi, logString, decFormatB, true); // 6 chars
    logString.append(", iKd: ");                             // 7 chars
    etl::format_spec decFormatC;
    decFormatC.width(1).fill('0');                           // [1 char]
    etl::to_string(internalKd, logString, decFormatC, true); // 1 char
    logString.append(", sP: ");                              // 7 chars

    etl::to_string(setpoint, logString, decFormatA, true); // 4 chars
    logString.append(", e: ");                             // 5 chars

    etl::format_spec boolFormatA;
    boolFormatA.binary().width(1).fill('0');               // [1 char]
    etl::to_string(enabled, logString, boolFormatA, true); // 1 char

    return logString;
};

/* Private Helper Functions */

/**
 * @see
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization
 */
void PID::initialize(float input, float currentOutput) {
    lastTime = micros();
    outputSum = currentOutput;
    lastInput = input;

    if (this->outputSum > this->outMax) {
        this->outputSum = this->outMax;
    } else if (this->outputSum < this->outMin) {
        this->outputSum = this->outMin;
    }
}
