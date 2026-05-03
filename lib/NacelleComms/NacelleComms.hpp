/**
 * @file NacelleComms.h
 * @brief ESP-NOW communication module for nacelle controller.
 *
 * Handles wireless communication between nacelle and load box.
 * Sends RPM data and receives control/state information.
 */

#ifndef NACELLE_COMMS_HPP
#define NACELLE_COMMS_HPP

#include "2026Core/TurbinePacket/TurbinePacket.hpp"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <etl/format_spec.h>
#include <etl/string.h>
#include <etl/to_string.h>

/**
 * @brief MAC address of the load box controller.
 */
extern const uint8_t LOADBOX_MAC[];

/**
 * @brief Transmission period in milliseconds.
 */
// const unsigned long NACELLE_COMMS_SEND_PERIOD_MS = 100;

/**
 * @brief Communication timeout threshold in milliseconds.
 */
// const unsigned long NACELLE_COMMS_TIMEOUT_MS = 1500;

class NacelleComms {
  public:
    static constexpr char *TAG = "NCO";
    static constexpr uint8_t wiFiChannel = 6;

    static QueueHandle_t priorityDataQueue;

    NacelleComms();

    bool begin();
    bool sendNacelleData(int16_t rpm, int16_t angularAccell_RPMPS);
    bool isLinkAlive() const;
    // uint8_t getRemoteState() const;
    // uint8_t getRemoteEstop() const;
    // uint16_t getRemoteActuatorPos() const;

    // TODO - improve this and null terminator may not be needed
    static constexpr uint_fast8_t LOG_STRING_SIZE =
        3 + 7 + 6 + ((8 + 7) * 2) + 7 + 6 + 8 + 7 + 1;
    /**
     * @brief Get at string that describes the current state of the PID instance
     * @returns the current state of the PID instance as a string
     */
    etl::string<LOG_STRING_SIZE> getLogString() const;

    struct LogData {
        uint_fast32_t txEvents;
        uint_fast32_t bytesSent;
        uint_fast32_t bytesNotSent;
        uint_fast32_t rxEvents;
        uint_fast32_t bytesReceived;
    };

    LogData getLogData() const;

  private:
    static NacelleComms *instance_;
    unsigned long lastSendTime_;
    unsigned long lastRxTime_;
    static uint_fast32_t txEvents; // DONE: check against last years code
    static uint_fast32_t bytesSent;
    static uint_fast32_t bytesNotSent;
    static uint_fast32_t rxEvents;
    static uint_fast32_t bytesReceived;
    bool linkAlive_;
    NacellePacket outgoingPacket_ = {0};
    LoadboxPacket incomingPacket_ = {0};

    // uint8_t remoteState_;
    // uint8_t remoteEstop_;

    esp_err_t setupPeer_();
    static void onDataSent_(const wifi_tx_info_t *tx_info,
                            esp_now_send_status_t status);
    static void onDataRecv_(const esp_now_recv_info_t *recv_info,
                            const uint8_t *data, int len);
};

#endif // NACELLE_COMMS_HPP
