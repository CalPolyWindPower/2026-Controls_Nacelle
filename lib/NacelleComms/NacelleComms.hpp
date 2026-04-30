/**
 * @file NacelleComms.h
 * @brief ESP-NOW communication module for nacelle controller.
 *
 * Handles wireless communication between nacelle and load box.
 * Sends RPM data and receives control/state information.
 */

#ifndef NACELLE_COMMS_HPP
#define NACELLE_COMMS_HPP

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "2026Core/TurbinePacket/TurbinePacket.hpp"

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
  static constexpr char* TAG = "NCO";
  static constexpr uint8_t wiFiChannel = 6;

  static QueueHandle_t priorityDataQueue;

  NacelleComms();
  
  bool begin();
  bool sendNacelleData(int16_t rpm);
  bool isLinkAlive() const;
  // uint8_t getRemoteState() const;
  // uint8_t getRemoteEstop() const;
  // uint16_t getRemoteActuatorPos() const;

private:
  static NacelleComms* instance_;
  unsigned long lastSendTime_;
  unsigned long lastRxTime_;
  // uint16_t remoteActuatorPos_;
  NacellePacket outgoingPacket_ = {0};
  LoadboxPacket incomingPacket_ = {0};
  bool linkAlive_;
  // uint8_t remoteState_;
  // uint8_t remoteEstop_;

  esp_err_t setupPeer_();
  static void onDataSent_(const wifi_tx_info_t *tx_info, esp_now_send_status_t status);
  static void onDataRecv_(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
};

#endif // NACELLE_COMMS_HPP
