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

  serprintf("RFM95 radio @ %d MHz\n", (int) RF95_FREQ);
}

void loop() {
  receivePacket();

  if (Serial.available() >= 1) {
    Serial.println("Getting serial command...");
    String command = Serial.readString();
    if (command.charAt(command.length() - 1) == '\n') {
      command.remove(command.length() - 1);
    }
    char command_arr[command.length() + 1];
    command.toCharArray(command_arr, command.length() + 1);
    sendCommand(command_arr);
  }
}

void receivePacket() {
  Serial.println("Attempting to receive");
  
  if (!rf95.waitAvailableTimeout(1000)) {
    Serial.println("RF95 not available");
    return;
  }
  
  Serial.println("RF95 available");
  uint8_t byteBuffer[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(byteBuffer);
  
  if (!rf95.recv(byteBuffer, &len)) {
    Serial.println("Receive failed");
    return;
  }
  
  if (!len) {
    Serial.println("Message length 0, dropping");
    return;
  }

  if (len < RH_RF95_MAX_MESSAGE_LEN / 2) {
    // This packet is probably an ACK
    serprintf(
      "Received response packet with length %d, string '%s', and values: ",
       len, (char*) byteBuffer
    );
    for (int i = 0; i < len; i++) {
      serprintf("%d, ", byteBuffer[i]);
    }
    Serial.println();
  }
  else {
    // This packet is probably a data packet
    int numDatapoints = (int) ((len - PKT_PREAMBLE_LEN) / 4);
    
    serprintf(
      "Received packet for team %d on profile %d half %d with length %d (%d datapoints): ",
      byteBuffer[PKT_IDX_TEAM_NUM],
      byteBuffer[PKT_IDX_PROFILE_NUM],
      byteBuffer[PKT_IDX_PROFILE_HALF],
      len,
      numDatapoints
    );

    for (int i = 0; i < len; i++) {
      serprintf("%d, ", byteBuffer[i]);
    }
    Serial.println();

    int i = PKT_PREAMBLE_LEN;

    Serial.print("= datapoints: ");
    while (i < len) {
      memcpy(bytesUnion.byteArray, byteBuffer + i, sizeof(long));
      serprintf("(%l, ", bytesUnion.longVal);
      i += sizeof(long);

      memcpy(bytesUnion.byteArray, byteBuffer + i, sizeof(float));
      serprintf("%f), ", bytesUnion.floatVal);
      i += sizeof(float);
    }
    Serial.println();
  }
}

void sendCommand(char *message) {
  for (int i = 0; i < COMMAND_SPAM_TIMES; i++) {
    rf95.send(message, strlen(message));
    rf95.waitPacketSent();
    receivePacket();

    delay(COMMAND_SPAM_DELAY);
  
    serprintf("'%s' (len=%d) command sent! Iteration: %d\n", message, strlen(message), i);
  }
}
