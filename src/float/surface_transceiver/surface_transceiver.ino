// SURFACE TRANSCEIVER sketch originally built from:
//  rf95_client.pde
//  -*- mode: C++ -*-
//
// REQUIRED LIBRARIES:
// RadioHead v1.122.1 by Mike McCauley
// Blue Robotics MS5837 Library v1.1.1 by BlueRobotics

#include <SPI.h>
#include <RH_RF95.h>
#include "rov_common.hpp"

#define COMMAND_SPAM_TIMES 5
#define COMMAND_SPAM_DELAY 500
#define SURFACE_PKT_RX_TIMEOUT 1000
#define MAX_RESPONSE_LEN RH_RF95_MAX_MESSAGE_LEN >> 1  // Max length for ACKs/NACKs

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  Serial.begin(115200);
  // Wait until serial console is open; remove if not tethered to computer
  while (!Serial) ;

  Serial.println("Surface Transceiver");
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.println("Feather RFM95 RX Test!\n");

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("RFM95 radio init failed");
    while (1);
  }
  Serial.println("RFM95 radio init OK!");

  // Defaults after init are: 434.0MHz, modulation GFSK_Rb250Fd250
  // +13dbM (for low power module), no encryption
  // But we override frequency
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  serialPrintf("RFM95 radio @ %d MHz\n", (int) RF95_FREQ);
}

void loop() {
  receivePacket();

  if (Serial.available() >= 1) {
    Serial.println("Getting serial command...");
    String command = Serial.readString();
    if (command.charAt(command.length() - 1) == '\n') {
      command.remove(command.length() - 1);
    }

    sendCommand(command);
  }
}

/**
 * Try to receive a packet for SURFACE_PKT_RX_TIMEOUT ms.
 * Return true if a packet was received and it was an ACK/NACK packet; else return false.
 * Print data from data packets (note we return false if data packets are successfully received).
 */
bool receivePacket() {
  Serial.println("Attempting to receive");

  if (!rf95.waitAvailableTimeout(SURFACE_PKT_RX_TIMEOUT)) {
    Serial.println("RF95 not available");
    return false;
  }

  Serial.println("RF95 available");
  byte byteBuffer[RH_RF95_MAX_MESSAGE_LEN];
  byte len = sizeof(byteBuffer);

  if (!rf95.recv(byteBuffer, &len)) {
    Serial.println("Receive failed");
    return false;
  }

  if (!len) {
    Serial.println("Message length 0, dropping");
    return false;
  }

  if (len < MAX_RESPONSE_LEN) {
    // This packet is probably an ACK/NACK
    serialPrintf(
      "Received response packet with length %d, string '%s', and values: ",
       len, (char*) byteBuffer
    );
    for (int i = 0; i < len; i++) {
      serialPrintf("%d, ", byteBuffer[i]);
    }
    Serial.println();

    return true;
  }
  else {
    // This packet is probably a data packet
    int numDatapoints = (int) ((len - (PKT_HEADER_LEN) >> 2));
    
    serialPrintf(
      "Received packet for team %d on profile %d half %d with length %d (%d datapoints): ",
      byteBuffer[PKT_IDX_TEAM_NUM],
      byteBuffer[PKT_IDX_PROFILE_NUM],
      byteBuffer[PKT_IDX_PROFILE_HALF],
      len,
      numDatapoints
    );

    for (int i = 0; i < len; i++) {
      serialPrintf("%d, ", byteBuffer[i]);
    }
    Serial.println();

    serialPrintf(
      "ROS:%d,%d,%d:",
      byteBuffer[PKT_IDX_TEAM_NUM],
      byteBuffer[PKT_IDX_PROFILE_NUM],
      byteBuffer[PKT_IDX_PROFILE_HALF]
    );
    for (int i = PKT_HEADER_LEN; i < len; ) {
      serialPrintf("%l,", * (unsigned long*) (byteBuffer + i));
      i += sizeof(long);

      serialPrintf("%f;", * (float*) (byteBuffer + i));
      i += sizeof(float);
    }
    Serial.println();
  }

  return false;
}

void sendCommand(String command) {
  byte commandBytes[command.length() + 1];
  command.getBytes(commandBytes, command.length() + 1);

  for (int i = 0; i < COMMAND_SPAM_TIMES; i++) {
    rf95.send(commandBytes, command.length() + 1);
    rf95.waitPacketSent();
    bool receivedACK = receivePacket();

    serialPrintf("'%s' (len=%d) command sent! Iteration: %d\n", commandBytes, command.length() + 1, i);

    if (receivedACK) {
      break;
    }

    delay(COMMAND_SPAM_DELAY);
  }
}
