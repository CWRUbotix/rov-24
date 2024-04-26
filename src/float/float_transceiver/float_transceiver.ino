// FLOAT TRANSCEIVER sketch originally built from:
//  rf95_client.pde
//  -*- mode: C++ -*-
//
// REQUIRED LIBRARIES:
// RadioHead v1.122.1 by Mike McCauley
// Blue Robotics MS5837 Library v1.1.1 by BlueRobotics

#include <SPI.h>
#include <RH_RF95.h>
#include "MS5837.h"
#include "rov_common.hpp"

int freeMemory() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0  ? (int)&__heap_start : (int) __brkval);  
}

// H-bridge direction control pins
#define MOTOR_PWM 6   // Leave 100% cycle for top speed
#define PUMP_PIN 9    // Set high for pump (CW when facing down)
#define SUCK_PIN 10   // Set high for suck (CCW when facing down)


// Limit switch pins
#define LIMIT_FULL  12  // Low when syringe is full
#define LIMIT_EMPTY 11  // Low when syringe is empty

#define TEAM_NUM 11

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 877.0

#define PACKET_LEN RH_RF95_MAX_MESSAGE_LEN
#define PACKET_PREAMBLE_LEN 3
#define PACKET_PAYLOAD_LEN PACKET_LEN - PACKET_PREAMBLE_LEN

#define SECOND 1000
#define PRESSURE_READ_INTERVAL 200

// All delays in ms
#define RELEASE_MAX   300000
#define SUCK_MAX      10000
#define DESCEND_TIME  10000
#define PUMP_MAX      10000
#define ASCEND_TIME   10000
#define TX_MAX        60000
#define ONE_HOUR      360000

#define WAIT 0
#define SUCK 1
#define PUMP 2
#define STOP 3

uint8_t overrideState = 0;

const uint8_t SCHEDULE_LENGTH = 11;
unsigned long SCHEDULE[SCHEDULE_LENGTH][2] = {
  // Wait for max <time> or until surface signal
  {WAIT, RELEASE_MAX },

  // Profile 1
  {SUCK, SUCK_MAX    },
  {WAIT, DESCEND_TIME},
  {PUMP, PUMP_MAX    },
  {WAIT, ASCEND_TIME },

  {WAIT, TX_MAX      },

  // Profile 2
  {SUCK, SUCK_MAX    },
  {WAIT, DESCEND_TIME},
  {PUMP, PUMP_MAX    },
  {WAIT, ASCEND_TIME },

  {WAIT, ONE_HOUR    },
};

uint8_t stage = 0;

unsigned long stageStartTime;
unsigned long previousPressureReadTime;


/************ Radio Setup ***************/

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


// Singleton instance of the pressure sensor driver
MS5837 pressureSensor;

// Buffers for pressures & time data w/preambles containing:
// 1 byte team number, 1 byte profile index (0x00, 0x01), and 1 byte isThisATimePacket (true == 0xFF)
// Every pressure is a 32-bit float stored as 4 uint8_t entries
// Every time is a 32-bit unsigned long stored as 4 uint8_t entries
// 3 bytes preamble + (62 * 4 = 248 readings or times) = 251 bytes total = PACKET_SIZE
uint8_t pressurePacket[PACKET_LEN];
uint8_t timePacket[PACKET_LEN];
int packetIndex = PACKET_PREAMBLE_LEN;

// Converts floats and longs to byte arrays via shared memory
union {
  float floatVal;
  unsigned long longVal;
  uint8_t byteArray[4];
} bytesUnion;

uint8_t profileNum = 0;

void setup() {
  Serial.begin(115200);
  // Wait until serial console is open; remove if not tethered to computer
  while (!Serial) ;
  
  Serial.println("Float Transceiver");
  Serial.println();

  stageStartTime = millis();
  previousPressureReadTime = millis();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(LIMIT_EMPTY, INPUT_PULLUP);
  pinMode(LIMIT_FULL,  INPUT_PULLUP);

  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(SUCK_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);

  digitalWrite(MOTOR_PWM, LOW);
  digitalWrite(SUCK_PIN, LOW);
  digitalWrite(PUMP_PIN, LOW);

  pressurePacket[0] = TEAM_NUM;
  timePacket[0] = TEAM_NUM;
  pressurePacket[2] = 0;
  timePacket[2] = 255;

  initRadio();
  initPressureSensor();
}


void loop() {

  // Move to next stage if necessary
  bool submergeReceived = receiveCommand();
//  Serial.print("Submerge: ");
//  Serial.println(submergeReceived);

  if (overrideState == SUCK) {
    if (digitalRead(LIMIT_FULL) == HIGH) {
      suck();
    }
    else {
      stop();
    }
    return;
  }
  else if (overrideState == PUMP) {
    if (digitalRead(LIMIT_EMPTY) == HIGH) {
      pump();
    }
    else {
      stop();
    }
    return;
  }
  
  // Read the pressure if we're profiling
  if (
    ((1 <= stage && stage <= 4) || (6 <= stage && stage <= 9)) &&
    millis() >= previousPressureReadTime + PRESSURE_READ_INTERVAL &&
    packetIndex < PACKET_LEN  // Stop recording if we overflow buffer
  ) {
    Serial.print("Reading: ");
    previousPressureReadTime = millis();

    pressureSensor.read();
    bytesUnion.floatVal = pressureSensor.pressure(); // 5.0
    Serial.print("Pressure: ");
    Serial.println(pressureSensor.pressure());
    memcpy(pressurePacket + packetIndex, bytesUnion.byteArray, sizeof(float));

    bytesUnion.longVal = millis();
    memcpy(timePacket + packetIndex, bytesUnion.byteArray, sizeof(long));
    
    Serial.print(pressurePacket[packetIndex]);
    Serial.print(", ");
    Serial.print(pressurePacket[packetIndex + 1]);
    Serial.print(", ");
    Serial.print(pressurePacket[packetIndex + 2]);
    Serial.print(", ");
    Serial.println(pressurePacket[packetIndex + 3]);
    packetIndex += sizeof(float);
    Serial.print(" >> ");
    Serial.println(freeMemory());
  }

  // Transmit the pressure buffer if we're surfaced
  if (stage == 5 || stage == 10) {
    transmitPressurePacket();
    delay(300);
  }

  if (
    millis() >= stageStartTime + SCHEDULE[stage][1] ||
    (SCHEDULE[stage][0] == WAIT && submergeReceived) ||
    (SCHEDULE[stage][0] == SUCK && !digitalRead(LIMIT_FULL)) ||
    (SCHEDULE[stage][0] == PUMP && !digitalRead(LIMIT_EMPTY))
  ) {
    Serial.print(stage);
    Serial.print(" ");
    Serial.print(SCHEDULE[stage][0]);
    Serial.print(" ");
    Serial.print(stageStartTime);
    Serial.print(" ");
    Serial.print(SCHEDULE[stage][1]);
    Serial.print(" ");
    Serial.print(millis());
    Serial.print(" ");
    Serial.print(millis() >= stageStartTime + SCHEDULE[stage][1]);
    Serial.println();

    // If we just finished our time surfaced
    if (stage == 5 || stage == 10) {
      packetIndex = PACKET_PREAMBLE_LEN;
      profileNum++;
    }

    stageStartTime = millis();
    stage++;
    stop();

    // If we signal a third profile, restart the schedule
    if (stage >= SCHEDULE_LENGTH) {
      stage = 1;
    }

    // Switch to sucking/pumping depending on the stage we're entering
    if (SCHEDULE[stage][0] == SUCK) {
      suck();
    }
    else if (SCHEDULE[stage][0] == PUMP) {
      pump();
    }

    Serial.println(stage);
  }
}

bool receiveCommand() {
  Serial.println("Receiving command...");
  if (rf95.waitAvailableTimeout(900)) {
    Serial.println("RF has signal");
    uint8_t uintBuf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(uintBuf);
    Serial.println(len);
    if (rf95.recv(uintBuf, &len)) {
      if (!len) return false;
      uintBuf[len] = 0;
      char *charBuf = (char*) uintBuf;
      
      Serial.print("Received [");
      Serial.print(len);
      Serial.print("]: '");
      Serial.print(charBuf);
      Serial.println("'");
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);

      Serial.print(charBuf[0]);
      Serial.print(", ");
      Serial.println(charBuf[1]);

      char response[32];
      bool shouldSubmerge = false;

      // Submerging
      // Invalid command
      // Overriding: pump
      // Overriding: suck

      if (strcmp(charBuf, "submerge") == 0) {
        strcpy(response, "ACK SUBMERGING");
        shouldSubmerge = true;
      }
      else if (strcmp(charBuf, "pump") == 0) {
        strcpy(response, "ACK PUMPING");
        overrideState = PUMP;
      }
      else if (strcmp(charBuf, "suck") == 0) {
        strcpy(response, "ACK SUCKING");
        overrideState = SUCK;
      }
      else if (strcmp(charBuf, "return") == 0) {
        strcpy(response, "ACK RETURNING TO SCHEDULE");
        overrideState = 0;
      }
      else {
        strcpy(response, "Invalid command");
        Serial.println("Invalid command");
      }

      rf95.send((uint8_t*) response, strlen(response));
      rf95.waitPacketSent();
      return shouldSubmerge;
    }
    else {
      Serial.println("Receive failed");
    }
  }

  return false;
}

void transmitPressurePacket() {
  Serial.print(" >> ");
  Serial.println(freeMemory());

  Serial.print("Sending pressure packet #");
  Serial.print(profileNum);
  Serial.print(" with content {");
  for (int p = 0; p < PACKET_LEN; p++) {
    Serial.print(pressurePacket[p]);
    Serial.print(", ");
  }
  Serial.println("}");

  Serial.print("Sending time packet #");
  Serial.print(profileNum);
  Serial.print(" with content {");
  for (int p = 0; p < PACKET_LEN; p++) {
    Serial.print(timePacket[p]);
    Serial.print(", ");
  }
  Serial.println("}");

//  Serial.print("Sizes: ");
//  Serial.print(RH_RF95_MAX_MESSAGE_LEN);
//  Serial.print(" ");
//  Serial.print(PACKET_PREAMBLE_LEN);
//  Serial.print(" ");
//  Serial.print(PACKET_PAYLOAD_LEN);
//  Serial.print(" ");
//  Serial.print(packetLength);
//  Serial.println();

  pressurePacket[1] = profileNum;
  timePacket[1] = profileNum;
  
  rf95.send(pressurePacket, PACKET_LEN);
  Serial.print(" >> ");
  Serial.println(freeMemory());
  rf95.waitPacketSent();
  
  rf95.send(timePacket, PACKET_LEN);
  Serial.print(" >> ");
  Serial.println(freeMemory());
  rf95.waitPacketSent();
  
  delay(1000);
}


void pump() {
  Serial.println("PUMPING");
  digitalWrite(MOTOR_PWM, HIGH);
  digitalWrite(PUMP_PIN, HIGH);
  digitalWrite(SUCK_PIN, LOW);
}

void suck() {
  Serial.println("SUCKING");
  digitalWrite(MOTOR_PWM, HIGH);
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(SUCK_PIN, HIGH);
}

void stop() {
  Serial.println("STOPPING");
  digitalWrite(MOTOR_PWM, LOW);
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(SUCK_PIN, LOW);
}


/******* Setup Methods (down here cause they're lorge) *******/


void initRadio() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.println("Feather RFM95 TX Test!");
  Serial.println();

  // Manually reset radio module
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
  }

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  Serial.print("RFM95 radio @ ");
  Serial.print((int) RF95_FREQ);
  Serial.println(" MHz");
}

void initPressureSensor() {
  pressureSensor.init();
  
  pressureSensor.setModel(MS5837::MS5837_02BA);
  pressureSensor.setFluidDensity(997);  // kg/m^3

  Serial.print("PRESSURE: ");
  Serial.println(pressureSensor.pressure());
}
