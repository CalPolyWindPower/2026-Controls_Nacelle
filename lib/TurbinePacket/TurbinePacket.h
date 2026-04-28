#ifndef TURBINE_PACKET_H
#define TURBINE_PACKET_H

#include <Arduino.h>

// Sender IDs
const uint8_t SENDER_NACELLE = 1;
const uint8_t SENDER_LOADBOX = 2;

// =====================
// NACELLE → LOADBOX
// =====================
struct NacellePacket {
  float rpm;
};

// =====================
// LOADBOX → NACELLE
// =====================
struct LoadboxPacket {
  uint8_t state;
  uint8_t estop;
  float actuatorPos;
};

// =====================
// Utility functions
// =====================

// Build packets
void makeNacellePacket(
 NacellePacket &packet,
 float rpm
 );

void makeLoadboxPacket(
  LoadboxPacket &packet,
  uint8_t state,
  uint8_t estop,
  uint16_t actuatorPos
);

// Debug printing
void printNacellePacket(const NacellePacket &packet, Stream &out);
void printLoadboxPacket(const LoadboxPacket &packet, Stream &out);

#endif