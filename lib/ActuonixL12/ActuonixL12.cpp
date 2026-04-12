#include "ActuonixL12.hpp"
#include <esp_log.h>

ActuonixL12::ActuonixL12(int pin, int min_us, int max_us)
    : pin_(pin), min_us_(min_us), max_us_(max_us) {}

void ActuonixL12::begin()
{
    ESP_LOGI(TAG, "Attaching on pin %d (range %d-%d us)", pin_, min_us_,
             max_us_);
    servo_.attach(pin_);
}

/**
 * @see https://stackoverflow.com/questions/3350385/how-to-return-an-object-in-c
 */
etl::string<ActuonixL12::LOG_STRING_SIZE> ActuonixL12::getLogString() {
    etl::string<LOG_STRING_SIZE> logString(TAG); // 3 chars
    logString.append(": Pus: ");                 // 6 chars

    etl::format_spec format;
    format.width(4).fill('0'); // [6 chars]
    etl::to_string(servo_.readMicroseconds(), logString, format,
                   true); // 6 chars

    return logString;
};

void ActuonixL12::writePosMicros(int us) {
    int clamped = us;

    if (clamped < min_us_)
        clamped = min_us_;
    if (clamped > max_us_)
        clamped = max_us_;

    if (clamped != us)
    {
        ESP_LOGW(TAG, "Clamped %d us -> %d us", us, clamped);
    }
    else
    {
        ESP_LOGD(TAG, "Command %d us", clamped);
    }

    servo_.writeMicroseconds(clamped);
}