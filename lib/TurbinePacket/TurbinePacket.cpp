#include "TurbinePacket.h"

// =====================
// Create packets
// =====================

void makeNacellePacket(NacellePacket &packet, float rpm) {
  packet.rpm = rpm;
}

void makeLoadboxPacket(
  LoadboxPacket &packet,
  uint8_t state,
  uint8_t estop,
  uint16_t actuatorPos
) {
  packet.state = state;
  packet.estop = estop;
  packet.actuatorPos = actuatorPos;
}

// =====================
// Debug printing
// =====================

void printNacellePacket(const NacellePacket &packet, Stream &out) {
  out.println("---- NacellePacket ----");
  out.print("RPM: ");
  out.println(packet.rpm);
  out.println("-----------------------");
}

void printLoadboxPacket(const LoadboxPacket &packet, Stream &out) {
  out.println("---- LoadboxPacket ----");
  out.print("State: ");
  out.println(packet.state);

  out.print("EStop: ");
  out.println(packet.estop);

  out.print("ActuatorPos: ");
  out.println(packet.actuatorPos);

  out.println("-----------------------");
}