#include "ActuonixL12.hpp"
#include <esp_log.h>

// MARK: Public
ActuonixL12::ActuonixL12(int pin, uint8_t feedbackPin, int min_us, int max_us)
    : pin_(pin), feedbackPin_(feedbackPin), min_us_(min_us), max_us_(max_us) {}

void ActuonixL12::begin() {
    ESP_LOGI(TAG, "Attaching on pin %d (range %d-%d us)", pin_, min_us_,
             max_us_);
    (void)servo_.attach(pin_);

    ESP_LOGI(TAG, "Configuring feedback pin %d", feedbackPin_);
    pinMode(feedbackPin_, INPUT);
    analogSetPinAttenuation(feedbackPin_, ADC_11db);
}

// MARK: Setters
void ActuonixL12::writePosMicros(int us) {
    int clamped = us;

    if (clamped < min_us_)
        clamped = min_us_;
    if (clamped > max_us_)
        clamped = max_us_;

    if (clamped != us) {
        ESP_LOGW(TAG, "Clamped %d us -> %d us", us, clamped);
    } else {
        ESP_LOGD(TAG, "Command %d us", clamped);
    }

    ESP_LOGD(TAG, "Writing %d us to servo on pin %d", clamped, pin_);
    servo_.writeMicroseconds(clamped);
}

// MARK: Getters
/**
 * @see https://stackoverflow.com/questions/3350385/how-to-return-an-object-in-c
 */
etl::string<ActuonixL12::LOG_STRING_SIZE> ActuonixL12::getLogString() {
    etl::string<LOG_STRING_SIZE> logString(TAG); // 3 chars
    (void)logString.append(": SP: ");            // 5 chars

    etl::format_spec format;
    (void)format.width(4).fill('0'); // [6 chars]
    etl::to_string(servo_.readMicroseconds(), logString, format,
                   true); // 6 chars

    (void)logString.append(": RP: "); // 5 chars
    etl::to_string(readPos_us(), logString, format,
                   true); // 6 chars

    return logString;
};

/**
 * @see
 * https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html
 */
uint_fast16_t ActuonixL12::readPos_mV() {
    return analogReadMilliVolts(feedbackPin_);
}

uint_fast16_t ActuonixL12::readPos_us() {
    return map(readPos_mV(), FEEDBACK_RANGE_mV.first, FEEDBACK_RANGE_mV.second,
               PULSE_RANGE_us.first, PULSE_RANGE_us.second);
}