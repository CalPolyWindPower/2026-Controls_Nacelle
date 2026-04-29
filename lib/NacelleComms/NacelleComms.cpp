/**
 * @file NacelleComms.cpp
 * @brief ESP-NOW communication module for nacelle controller.
 *
 * Handles wireless communication between nacelle and load box.
 * Sends RPM data and receives control/state information.
 */

#include "NacelleComms.hpp"
#include <esp_log.h>

// Initialization of static members
QueueHandle_t NacelleComms::priorityDataQueue = nullptr;

/**
 * @brief MAC address of the load box controller.
 */
const uint8_t LOADBOX_MAC[] = {0xEC, 0xDA, 0x3B, 0x5C, 0x93, 0x04};


/**
 * @brief Static pointer to the current NacelleComms instance for callback access.
 */
NacelleComms* NacelleComms::instance_ = nullptr;


/**
 * @brief Constructor. Initializes member variables and sets the singleton instance pointer.
 */
NacelleComms::NacelleComms()
    : lastSendTime_(0),
      lastRxTime_(0),
      linkAlive_(false) //,
      // remoteState_(0),
      // remoteEstop_(0),
      // remoteActuatorPos_(0.0f) 
      {
  instance_ = this;
  priorityDataQueue = xQueueCreate( 1, sizeof(LoadboxPacket) );
  if(priorityDataQueue == NULL) {
    ESP_LOGE(TAG, "Failed to create Rx priority data queue");
  }
}


/**
 * @brief Initializes ESP-NOW communication and registers callbacks.
 * @return true if initialization is successful, false otherwise.
 */
bool NacelleComms::begin() {
  // Set device as a Wi-Fi Station
    if (!WiFi.mode(WIFI_STA)) {
        ESP_LOGE(TAG, "Failed to set WiFi mode");
        return false;
    }

    if (!WiFi.setBandMode(WIFI_BAND_MODE_2G_ONLY)) {
        ESP_LOGE(TAG, "Failed to set WiFi band mode");
        return false;
    }

    if (WiFi.STA.bandwidth(WIFI_BW_HT20)) {
        ESP_LOGE(TAG, "Failed to set WiFi bandwidth");
        return false;
    }

    if (WiFi.setChannel(wiFiChannel) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi channel");
        return false;
    }

  if (esp_now_init() != ESP_OK) {
    ESP_LOGE(TAG, "ESP-NOW init failed");
    return false;
  }

  if(esp_now_register_send_cb(onDataSent_) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register tx cb");
    return false;
  }
  if(esp_now_register_recv_cb(onDataRecv_) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register rx cb");
    return false;
  }

  if( setupPeer_() != ESP_OK) {
    // Logging already handled
    return false;
  }

  ESP_LOGI(TAG, "Nacelle ready");
  return true;
}


/**
 * @brief Configures the ESP-NOW peer (load box controller).
 */
esp_err_t NacelleComms::setupPeer_() {
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, LOADBOX_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_err_t result = esp_now_add_peer(&peerInfo);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add peer: %d at %02x:%02x:%02x:%02x:%02x:%02x", result,
             LOADBOX_MAC[0], LOADBOX_MAC[1], LOADBOX_MAC[2],
             LOADBOX_MAC[3], LOADBOX_MAC[4], LOADBOX_MAC[5]);
  } else{
    ESP_LOGI(TAG, "Peer added: %02X:%02X:%02X:%02X:%02X:%02X",
           LOADBOX_MAC[0], LOADBOX_MAC[1], LOADBOX_MAC[2],
           LOADBOX_MAC[3], LOADBOX_MAC[4], LOADBOX_MAC[5]);
  }

  return result;
}


/**
 * @brief Callback executed after data is sent.
 * @param tx_info Transmission metadata.
 * @param status Transmission result.
 */
void NacelleComms::onDataSent_(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  // (void)tx_info;
  // Serial.print("Send status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
  // This is check elsewhere
}


/**
 * @brief Callback executed when data is received from the load box.
 * @param recv_info Receive metadata including sender MAC.
 * @param data Pointer to received data buffer.
 * @param len Length of received data.
 */
void NacelleComms::onDataRecv_(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (instance_ == nullptr) {
    ESP_LOGE(TAG, "Rx CB: Invalid instance");
    return;
  }
  if (len == sizeof(LoadboxPacket)) {
    // memcpy(&instance_->incomingPacket_, data, sizeof(LoadboxPacket));
    instance_->lastRxTime_ = millis();
    instance_->linkAlive_ = true;
    // instance_->remoteState_ = instance_->incomingPacket_.state;
    // instance_->remoteEstop_ = instance_->incomingPacket_.estop;
    // instance_->remoteActuatorPos_ = instance_->incomingPacket_.actuatorPos;
    ESP_LOGI(TAG, "Rx success");
    // printLoadboxPacket(instance_->incomingPacket_, Serial);

    (void)xQueueOverwrite(priorityDataQueue, data); // Allegedly cannot fail
  } else {
    ESP_LOGE(TAG, "Rx invalid len: %d", len);
  }
}


/**
 * @brief Sends RPM data to the load box controller.
 * @param rpm The RPM value to send.
 * @returns true if send is initiated successfully, false otherwise.
 */
bool NacelleComms::sendNacelleData(int16_t rpm) {
  // if (now - lastSendTime_ >= NACELLE_COMMS_SEND_PERIOD_MS) {
  makeNacellePacket(outgoingPacket_, rpm);
  esp_err_t result = esp_now_send(LOADBOX_MAC, (uint8_t *)&outgoingPacket_, sizeof(outgoingPacket_));
  if(result == ESP_OK) {
    lastSendTime_ = millis();
    linkAlive_ = true;
  } else {
    linkAlive_ = false;
    ESP_LOGE(TAG, "Tx failed");
  }
  // }
  // if (now - lastRxTime_ > NACELLE_COMMS_TIMEOUT_MS) {
  //   linkAlive_ = false;
  // }
  // if (!linkAlive_) {
  //   Serial.println("WARNING: load-box comms timeout");
  // }
  return linkAlive_;
}


/**
 * @brief Checks if the communication link is alive.
 * @return true if link is alive, false otherwise.
 */
bool NacelleComms::isLinkAlive() const {
  return linkAlive_;
}


/**
 * @brief Gets the latest received state from the load box.
 * @return State value.
 */
// uint8_t NacelleComms::getRemoteState() const {
//   return remoteState_;
// }

/**
 * @brief Gets the latest received E-stop value from the load box.
 * @return E-stop value.
 */
// uint8_t NacelleComms::getRemoteEstop() const {
//   return remoteEstop_;
// }

/**
 * @brief Gets the latest received actuator position from the load box.
 * @return Actuator position value.
 */
// uint16_t NacelleComms::getRemoteActuatorPos() const {
//   return remoteActuatorPos_;
// }
