#include "ActuonixL12.hpp"
#include <esp_log.h>

static const char *TAG = "ActL12";

ActuonixL12::ActuonixL12(int pin, int min_us, int max_us)
    : pin_(pin), min_us_(min_us), max_us_(max_us) {}

void ActuonixL12::begin()
{
    ESP_LOGI(TAG, "Attatching on pin %d (range %d-%d us)", pin_, min_us_, max_us_);
    servo_.attach(pin_);
}

void ActuonixL12::writePosMicros(int us)
{
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