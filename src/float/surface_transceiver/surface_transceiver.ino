// SURFACE TRANSCEIVER sketch originally built from:
//  rf95_client.pde
//  -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf95_server
// Tested with Anarduino MiniWirelessLoRa, Rocket Scream Mini Ultra Pro with
// the RFM95W, Adafruit Feather M0 with RFM95

#include <SPI.h>
#include <RH_RF95.h>

#if defined (__AVR_ATmega32U4__)  // Feather 32u4 w/Radio
  #define RFM95_CS    8
  #define RFM95_INT   7
  #define RFM95_RST   4

#elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)  // Feather M0 w/Radio
  #define RFM95_CS    8
  #define RFM95_INT   3
  #define RFM95_RST   4

#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_RFM)  // Feather RP2040 w/Radio
  #define RFM95_CS   16
  #define RFM95_INT  21
  #define RFM95_RST  17

#elif defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM95_CS    4  //
  #define RFM95_INT   3  //
  #define RFM95_RST   2  // "A"

#elif defined(ESP8266)  // ESP8266 feather w/wing
  #define RFM95_CS    2  // "E"
  #define RFM95_INT  15  // "B"
  #define RFM95_RST  16  // "D"

#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_NRF52840_FEATHER) || defined(ARDUINO_NRF52840_FEATHER_SENSE)
  #define RFM95_CS   10  // "B"
  #define RFM95_INT   9  // "A"
  #define RFM95_RST  11  // "C"

#elif defined(ESP32)  // ESP32 feather w/wing
  #define RFM95_CS   33  // "B"
  #define RFM95_INT  27  // "A"
  #define RFM95_RST  13

#elif defined(ARDUINO_NRF52832_FEATHER)  // nRF52832 feather w/wing
  #define RFM95_CS   11  // "B"
  #define RFM95_INT  31  // "C"
  #define RFM95_RST   7  // "A"

#endif

/************ Radio Setup ***************/

#define PACKET_PREAMBLE_LEN 3

// Change to 434.0 or other frequency, must match float's freq!
#define RF95_FREQ 877.0

#define COMMAND_MAX_LEN 50

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Converts byte arrays to floats via shared memory
union {
  float floatVal;
  unsigned long longVal;
  uint8_t byteArray[4];
} bytesUnion;

// Store the last command ordered over serial
// Spam the float with this until it ACKs
char pendingCommand[COMMAND_MAX_LEN];

void setup() {
  Serial.begin(115200);
  // Wait until serial console is open; remove if not tethered to computer
  while (!Serial) ;

  Serial.println("Surface Transceiver");
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.println("Feather RFM95 RX Test!");
  Serial.println();

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

  Serial.print("RFM95 radio @ ");
  Serial.print((int) RF95_FREQ);
  Serial.println(" MHz");
}

void loop() {
  receivePacket();

  if (Serial.available() >= 1) {
    String command;
    Serial.println("Getting serial command...");
    command = Serial.readString();
    Serial.println(command);
    if (command == "submerge" || command == "submerge\n") {
      sendCommand("submerge");
      receivePacket();
    }
    else if (command == "submerge" || command == "submerge\n") {
      sendCommand("submerge");
      receivePacket();
    }
    else {
      Serial.println("Illegal command; sending anyway");
      char command_arr[command.length() + 1];
      command.toCharArray(command_arr, command.length() + 1);
      sendCommand(command_arr);
    }
    // else if (command == "extend") {
    //   sendCommand("extend");
    // }
    // else if (command == "retract") {
    //   sendCommand("retract");
    // }

  }
}

void receivePacket() {
  Serial.println("Attempting to receive");
  if (rf95.waitAvailableTimeout(1000)) {
    Serial.println("RF95 available");
    uint8_t byteBuffer[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(byteBuffer);
    if (rf95.recv(byteBuffer, &len)) {
      if (!len) {
        Serial.println("Message length 0, dropping");
        return;
      }

      if (len < RH_RF95_MAX_MESSAGE_LEN / 2) {
        // This packet is probably an ACK
        Serial.print("Received message packet with length ");
        Serial.print(len);
        Serial.print(", string '");
        byteBuffer[len] = 0;
        char *charBuf = (char*) byteBuffer;
        Serial.print(charBuf);
        Serial.print("', and values: ");
        for (int i = 0; i < len; i++) {
          Serial.print(byteBuffer[i]);
          Serial.print(", ");
        }
        Serial.println();

        if (strcmp(pendingCommand, "ACK") == 0) {
          pendingCommand[0] = 0;
        }
      }
      else {
        // This packet is probably a data packet
        bool isTimePacket = byteBuffer[2];
        
        Serial.print("Received ");
        Serial.print(isTimePacket ? "time" : "pressure");
        Serial.print(" packet for team ");
        Serial.print(byteBuffer[0]);
        Serial.print(" on profile ");
        Serial.print(byteBuffer[1]);
        Serial.print(" with length ");
        Serial.print(len);
        Serial.print(": ");
        for (int i = 0; i < len; i++) {
          Serial.print(byteBuffer[i]);
          Serial.print(", ");
        }
        Serial.println();
  
        if (isTimePacket) {
          Serial.print("= longs with length ");
          Serial.print((len - PACKET_PREAMBLE_LEN) / sizeof(long));
          Serial.print(": ");
          for (int i = PACKET_PREAMBLE_LEN; i < len; i += sizeof(long)) {
            memcpy(bytesUnion.byteArray, byteBuffer + i, sizeof(long));
            Serial.print(bytesUnion.longVal);
            Serial.print(", ");
          }
        }
        else {
          Serial.print("= floats with length ");
          Serial.print((len - PACKET_PREAMBLE_LEN) / sizeof(float));
          Serial.print(": ");
          for (int i = PACKET_PREAMBLE_LEN; i < len; i += sizeof(float)) {
            memcpy(bytesUnion.byteArray, byteBuffer + i, sizeof(float));
            Serial.print(bytesUnion.floatVal);
            Serial.print(", ");
          }
        }
        Serial.println();
      }
    }
    else {
      Serial.println("Receive failed");
    }
  }
  else {
    Serial.println("RF95 not available"); 
  }
}

void sendCommand(char *message) {
  strcpy(pendingCommand, message);
  
  rf95.send(message, strlen(message));
  rf95.waitPacketSent();
  Serial.print(message);
  Serial.print(strlen(message));
  Serial.println(" signal sent!");
}
