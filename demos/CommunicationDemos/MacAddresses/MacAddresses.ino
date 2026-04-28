#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  delay(100);

  Serial.println();
  Serial.println("ESP32 MAC Address Finder");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
}