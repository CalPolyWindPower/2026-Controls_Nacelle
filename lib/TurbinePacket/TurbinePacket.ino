// Packet Demo

#include "TurbinePacket.hpp"

NacellePacket packet1;
LoadboxPacket packet2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  makeNacellePacket(packet1, 1500);
  makeLoadboxPacket(packet2, 1, 1, 1);
}

void loop() {
  // put your main code here, to run repeatedly:
  printNacellePacket(packet1, Serial);
  printLoadboxPacket(packet2, Serial);
}
