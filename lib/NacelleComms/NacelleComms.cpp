/**
 * @file NacelleComms.cpp
 * @brief ESP-NOW communication module for nacelle controller.
 *
 * Handles wireless communication between nacelle and load box.
 * Sends RPM data and receives control/state information.
 */

#include "NacelleComms.hpp"
#include <esp_log.h>
#include <esp_wifi.h>

// Initialization of static members
QueueHandle_t NacelleComms::priorityDataQueue = nullptr;
uint_fast32_t NacelleComms::txEvents = 0; // DONE: check against last years code
uint_fast32_t NacelleComms::bytesSent = 0;
uint_fast32_t NacelleComms::bytesNotSent = 0;
uint_fast32_t NacelleComms::rxEvents = 0;
uint_fast32_t NacelleComms::bytesReceived = 0;

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
    
    etl::array<uint8_t, 6> MACAddress = {0};
    esp_err_t opStatus = esp_wifi_get_mac(WIFI_IF_STA, MACAddress.data());
    if(opStatus != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get MAC address: %d", opStatus);
        return false;
    } else {
        ESP_LOGI(TAG, "Device MAC: %02X:%02X:%02X:%02X:%02X:%02X",
               MACAddress[0], MACAddress[1], MACAddress[2],
               MACAddress[3], MACAddress[4], MACAddress[5]);
    }


    if (!WiFi.STA.bandwidth(WIFI_BW_HT20)) {
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
  (void)memcpy(peerInfo.peer_addr, LOADBOX_MAC, 6);
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
  txEvents++;
  if(status == ESP_NOW_SEND_SUCCESS) {
    bytesSent += tx_info->data_len;
  } else {
    bytesNotSent += tx_info->data_len;
  }
  
}


/**
 * @brief Callback executed when data is received from the load box.
 * @param recv_info Receive metadata including sender MAC.
 * @param data Pointer to received data buffer.
 * @param len Length of received data.
 */
void NacelleComms::onDataRecv_(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  rxEvents++;
  bytesReceived += len;
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
    // ESP_LOGI(TAG, "Rx success");
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
  esp_err_t result = esp_now_send(LOADBOX_MAC, reinterpret_cast<uint8_t *>(&outgoingPacket_), sizeof(outgoingPacket_));
  if(result == ESP_OK) {
    lastSendTime_ = millis();
    linkAlive_ = true;
  } else {
    linkAlive_ = false;
    // ESP_LOGE(TAG, "Tx failed");
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

etl::string<NacelleComms::LOG_STRING_SIZE> NacelleComms::getLogString() const {
        etl::string<LOG_STRING_SIZE> logString(TAG); // 3 chars
        logString.append(": TxE: ");                 // 7 chars

        etl::format_spec decFormatA;
        decFormatA.width(6).fill('0'); // [6 chars, expecting up to 30 mins * (1/(2ms)) = 900,000 events]
        /**
         * @details I don't think we need strong guarantees on logging data
         * @see
         * https://stackoverflow.com/questions/12346487/what-do-each-memory-order-mean
         * @see https://en.cppreference.com/cpp/atomic/memory_order
         */
        etl::to_string(txEvents, logString, decFormatA, true); // 6 chars
        logString.append(", TxBS: "); // 8 chars

        etl::format_spec decFormatB;
        decFormatB.width(7).fill('0'); // [7 chars]
        etl::to_string(bytesSent, logString, decFormatB, true); // 7 chars
        logString.append(", TxBF: ");                 // 8 chars
        etl::to_string(bytesNotSent, logString, decFormatB, true); // 7 chars

        logString.append(", RxE: ");                 // 7 chars
        etl::to_string(rxEvents, logString, decFormatA, true); // 6 chars
        logString.append(", RxBS: "); // 8 chars
        etl::to_string(bytesReceived, logString, decFormatB, true); // 7 chars

        return logString;
    }

