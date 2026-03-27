#pragma once
#include <ESP32Servo.h>

class ActuonixL12
{
public:
    explicit ActuonixL12(int pin, int min_us = 1000, int max_us = 2000);
    ~ActuonixL12() = default;

    void begin();
    void writePosMicros(int us);

private:
    Servo servo_;
    int pin_;
    int min_us_;
    int max_us_;
};
