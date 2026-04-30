#pragma once

#include <Arduino.h>
#include <cctype>

static SerialInterface *instance_;

class SerialInterface {
  public:
    static constexpr const char *TAG = "serialInterface";

    SerialInterface(int baudRate_)
        : baudRate(baudRate_) {

        static bool serialInitialized = false;
        if (!serialInitialized) {
            Serial.begin(baudRate_);
            ESP_LOGI(TAG, "Serial initialized; baud rate: %d", baudRate_);
            serialInitialized = true;
        }
    }

    float getKp() const { return kp; }

    float getKi() const { return ki; }

    float getKd() const { return kd; }
    
    float getPosition() const { return position; }

    String getCommand() const { return command; }

    void parse() {
        String line = Serial.readStringUntil('\n');
        line.trim();
        stringArray parts = split(line, ' ');

        if (parts.count > 0) {
            command = parts.tokens[0];
        }
    }

  private:
    int baudRate;
    int position = 0;
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    String command = "No Command";

    struct stringArray {
        String tokens[10]; // Max 10 tokens
        int count;
    };

    /**
     * @brief Split a String by a delimiter character.
     * @param str The String to split
     * @param delimiter The character to split on
     * @return Array of String tokens
     */
    stringArray split(const String &str, char delimiter) {
        stringArray output;
        int start = 0;

        for (int i = 0; i <= str.length(); i++) {
            if (i == str.length() || str.charAt(i) == delimiter) {
                if (i > start && output.count < 10) {
                    output.tokens[output.count++] = str.substring(start, i);
                }
                start = i + 1;
            }
        }

        return output;
    }
};