#pragma once

#include <Arduino.h>
#include <cctype>
#include "NacelleTasks.hpp"
#include "NacelleContainer.hpp"
#include "NacelleConfig.hpp"
//using TaskFunction = void (*)(void *);

class SerialInterface {
  public:
    static constexpr const char *TAG = "serialInterface";

    SerialInterface(int baudRate_, NacelleContainer &nacelle, NacelleFSM &nacelleFSM)
        : baudRate(baudRate_),
          nacelle(nacelle),
          nacelleFSM(nacelleFSM) {
    }

    void begin() {
        Serial.begin(baudRate);
        ESP_LOGI(TAG, "SerialInterface Initialized at baud rate %d", baudRate);
    }
    
    float getPosition() const { return position; }

    String getCommand() const { return command; }

    void parse() {
        String line = Serial.readStringUntil('\n');
        line.trim();
        ESP_LOGI(TAG, "%s", line.c_str());
        stringArray parts = split(line, ' ');

        if (parts.count > 0) {
            command = parts.tokens[0];
            if (command.equalsIgnoreCase("pid")) {
                Kp = parts.tokens[1].toFloat();
                Ki = parts.tokens[2].toFloat();
                Kd = parts.tokens[3].toFloat();
                nacelle.pitchPIDController.setTunings(Kp, Ki, Kd);
                ESP_LOGI(TAG, "Updated PID constants: Kp=%.2f, Ki=%.2f, Kd=%.2f", Kp, Ki, Kd);
            }
            else if (command.equalsIgnoreCase("Kp")) {
                Kp = parts.tokens[1].toFloat();
                ESP_LOGI(TAG, "Updated Kp: %.2f", Kp);
                nacelle.pitchPIDController.setTunings(Kp, Ki, Kd);
            }
            else if (command.equalsIgnoreCase("Ki")) {
                Ki = parts.tokens[1].toFloat();
                ESP_LOGI(TAG, "Updated Ki: %.2f", Ki);
                nacelle.pitchPIDController.setTunings(Kp, Ki, Kd);
            }
            else if (command.equalsIgnoreCase("Kd")) {
                Kd = parts.tokens[1].toFloat();
                ESP_LOGI(TAG, "Updated Kd: %.2f", Kd);
                nacelle.pitchPIDController.setTunings(Kp, Ki, Kd);
            }
            else if (command.equalsIgnoreCase("position")) {
                position = parts.tokens[1].toInt();
                nacelle.pitchActuator.writePosMicros(position);
                ESP_LOGI(TAG, "Updated position: %d", position);
            }
            else if (command.equalsIgnoreCase("disableFSM")) {
                FSM_ENABLED = false;
                vTaskSuspend(nacelle.mainTaskDescriptions[0].pxHandle); // Suspend FSM task to reset state
                ESP_LOGI(TAG, "Disabled FSM");
            }
            else if (command.equalsIgnoreCase("enableFSM")) {
                FSM_ENABLED = true;
                vTaskResume(nacelle.mainTaskDescriptions[0].pxHandle); // Resume FSM
                ESP_LOGI(TAG, "Enabled FSM");
            }
            else if (command.equalsIgnoreCase("enablePID")) {
                nacelle.pitchPIDController.enable(nacelle.currentRPM, PITCHING::POS_RUN_uS);
                ESP_LOGI(TAG, "Enabled PID controller");
            }
            else if (command.equalsIgnoreCase("disablePID")) {
                nacelle.pitchPIDController.disable();
                ESP_LOGI(TAG, "Disabled PID controller");
            }
            else if (command.equalsIgnoreCase("updateSetpoint")){
                setpoint = parts.tokens[1].toFloat();
                nacelle.pitchPIDController.setTarget(setpoint);
                ESP_LOGI(TAG, "Updated setpoint: %.2f", setpoint);
            }
            else if (command.equalsIgnoreCase("disableSafety")) {
                // This is a bit of a hack, but it allows us to test the FSM without triggering safety conditions
                nacelle.setEnableSafetyFlag(false);
                ESP_LOGI(TAG, "Disabled safety (ESTOP)");
            }
             else if (command.equalsIgnoreCase("enableSafety")) {
                // This is a bit of a hack, but it allows us to test the FSM without triggering safety conditions
                nacelle.setEnableSafetyFlag(true);
                ESP_LOGI(TAG, "Enabled safety (OVERSPEED ESTOP)");
            }
            else if (command.equalsIgnoreCase("enableRpmOutput")) {
                PITCHING::enableRpmOutput = true;
                ESP_LOGI(TAG, "Enabled RPM output to log");
            }
            else if (command.equalsIgnoreCase("disableRpmOutput")) {
                PITCHING::enableRpmOutput = false;
                ESP_LOGI(TAG, "Disabled RPM output to log");

            }
            else if (command.equalsIgnoreCase("setTargetPower")) {
                nacelleFSM.setTargetPower(parts.tokens[1].toFloat());
                ESP_LOGI(TAG, "Updated target power: %.2f", nacelleFSM.targetPower_mW);
            }
            else if (command.equalsIgnoreCase("help")) {
                Serial.println("Available commands:");
                Serial.println("  pid <Kp> <Ki> <Kd> - Set all PID constants");
                Serial.println("  Kp <value> - Set Kp constant");
                Serial.println("  Ki <value> - Set Ki constant");
                Serial.println("  Kd <value> - Set Kd constant");
                Serial.println("  position <value> - Set actuator position in microseconds");
                Serial.println("  enableFSM - Enable the FSM task");
                Serial.println("  disableFSM - Disable the FSM task");
                Serial.println("  enablePID - Enable the PID controller");
                Serial.println("  disablePID - Disable the PID controller");
                Serial.println("  updateSetpoint <value> - Update the PID setpoint");
                Serial.println("  enableSafety - Enable safety flag (ESTOP)");
                Serial.println("  disableSafety - Disable safety flag (ESTOP)");
            }
            else {
                ESP_LOGE(TAG, "Unknown command: %s", command.c_str());
            }
        }
        else {
            ESP_LOGE(TAG, "Invalid, no input received");
        }     
        
    }


  private:
    static SerialInterface *instance_;
    int baudRate;
    NacelleContainer &nacelle;
    NacelleFSM &nacelleFSM;
    int position = 0;
    bool FSM_ENABLED = true;
    String command = "";
    float Kp = 0.0f;
    float Ki = 0.0f;
    float Kd = 0.0f;
    float setpoint = 0.0f;

    struct stringArray {
        String tokens[10]; // Max 10 tokens
        int count = 0;
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