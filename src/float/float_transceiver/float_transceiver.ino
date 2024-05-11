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

// H-bridge direction control pins
#define MOTOR_PWM 6   // Leave 100% cycle for top speed
#define PUMP_PIN 9    // Set high for pump (CW when facing down)
#define SUCK_PIN 10   // Set high for suck (CCW when facing down)


// Limit switch pins
#define LIMIT_FULL  12  // Low when syringe is full
#define LIMIT_EMPTY 11  // Low when syringe is empty


#define TEAM_NUM 11
#define PRESSURE_READ_INTERVAL 200
#define PACKET_SEND_INTERVAL   1000


// Schedule (all delays in ms)
#define RELEASE_MAX   300000
#define SUCK_MAX      10000
#define DESCEND_TIME  10000
#define PUMP_MAX      10000
#define ASCEND_TIME   10000
#define TX_MAX        60000
#define ONE_HOUR      360000

#define NO_OVERRIDE 0
#define WAIT_PROFILING 0
#define WAIT_SURFACED 1
#define SUCK 2
#define PUMP 3
#define STOP 4

#define SCHEDULE_LENGTH 11

uint8_t overrideState = 0;
uint8_t currentStage = 0;

unsigned long SCHEDULE[SCHEDULE_LENGTH][2] = {
  // Wait for max <time> or until surface signal
  {WAIT_SURFACED,  RELEASE_MAX },

  // Profile 1
  {SUCK,           SUCK_MAX    },
  {WAIT_PROFILING, DESCEND_TIME},
  {PUMP,           PUMP_MAX    },
  {WAIT_PROFILING, ASCEND_TIME },

  {WAIT_SURFACED,  TX_MAX      },

  // Profile 2
  {SUCK,           SUCK_MAX    },
  {WAIT_PROFILING, DESCEND_TIME},
  {PUMP,           PUMP_MAX    },
  {WAIT_PROFILING, ASCEND_TIME },

  {WAIT_SURFACED,  ONE_HOUR    },
};

unsigned long stageStartTime;
unsigned long previousPressureReadTime;
unsigned long previousPacketSendTime;


// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Singleton instance of the pressure sensor driver
MS5837 pressureSensor;

uint8_t pressurePacket[PKT_LEN];
uint8_t timePacket[PKT_LEN];
int packetIndex = PKT_PREAMBLE_LEN;

uint8_t profileNum = 0;

void setup() {
  Serial.begin(115200);
  // Wait until serial console is open; remove if not tethered to computer
//  while (!Serial) ;
  
  Serial.println("Float Transceiver");
  Serial.println();

  stageStartTime = millis();
  previousPressureReadTime = millis();
  previousPacketSendTime = millis();

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

  pressurePacket[PKT_IDX_TEAM_NUM] = TEAM_NUM;
  timePacket[PKT_IDX_TEAM_NUM] = TEAM_NUM;
  pressurePacket[PKT_IDX_IS_TIME] = 0;
  timePacket[PKT_IDX_IS_TIME] = 255;

  initRadio();
  initPressureSensor();
}


void loop() {

  // Move to next stage if necessary
  bool submergeReceived = receiveCommand();

  if (overrideState == SUCK) {
    if (digitalRead(LIMIT_FULL) == HIGH) {
      suck();
    }
    else {
      stop();
      overrideState = NO_OVERRIDE;
    }
    return;
  }
  else if (overrideState == PUMP) {
    if (digitalRead(LIMIT_EMPTY) == HIGH) {
      pump();
    }
    else {
      stop();
      overrideState = NO_OVERRIDE;
    }
    return;
  }
  
  // Read the pressure if we're profiling
  if (
    SCHEDULE[currentStage][0] != WAIT_SURFACED &&
    millis() >= previousPressureReadTime + PRESSURE_READ_INTERVAL &&
    packetIndex < PKT_LEN  // Stop recording if we overflow buffer
  ) {
    previousPressureReadTime = millis();

    pressureSensor.read();
    bytesUnion.floatVal = pressureSensor.pressure();
    serprintf("Reading pressure: %f\n", pressureSensor.pressure());
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
  }

  // Transmit the pressure buffer if we're surfaced
  if (
    SCHEDULE[currentStage][0] == WAIT_SURFACED &&
    millis() >= previousPacketSendTime + PACKET_SEND_INTERVAL
  ) {
    transmitPressurePacket();
    previousPacketSendTime = millis();
  }

  if (
    millis() >= stageStartTime + SCHEDULE[currentStage][1] ||
    (SCHEDULE[currentStage][0] == WAIT_SURFACED && submergeReceived) ||
    (SCHEDULE[currentStage][0] == SUCK && !digitalRead(LIMIT_FULL)) ||
    (SCHEDULE[currentStage][0] == PUMP && !digitalRead(LIMIT_EMPTY))
  ) {
    serprintf(
      "Stage #%d, type: %d, start time: %l, max wait time: %l, current time: %l\n",
      currentStage,
      SCHEDULE[currentStage][0],
      stageStartTime,
      SCHEDULE[currentStage][1],
      millis()
    );

    // If we just finished our time surfaced
    if (currentStage == 5 || currentStage == 10) {
      packetIndex = PKT_PREAMBLE_LEN;
      profileNum++;
    }

    stageStartTime = millis();
    currentStage++;
    stop();

    // If we signal a third profile, restart the schedule
    if (currentStage >= SCHEDULE_LENGTH) {
      currentStage = 1;
    }

    // Switch to sucking/pumping depending on the stage we're entering
    if (SCHEDULE[currentStage][0] == SUCK) {
      suck();
    }
    else if (SCHEDULE[currentStage][0] == PUMP) {
      pump();
    }

    Serial.println(currentStage);
  }
}

bool receiveCommand() {
  Serial.println("Receiving command...");
  
  if (!rf95.waitAvailableTimeout(900)) {
    return false;
  }
  
  Serial.println("RF has signal");
  uint8_t uintBuf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(uintBuf);
  
  if (!rf95.recv(uintBuf, &len)) {
    Serial.println("Receive failed");
    return false;
  }
  if (!len) {
    Serial.println("Received with length 0; dropping");
    return false;
  }
  
  uintBuf[len] = 0;
  char *charBuf = (char*) uintBuf;

  serprintf("Received [%d]: '%s'\n", len, charBuf);
  
  char response[32];
  bool shouldSubmerge = false;

  // TODO: Use a string or add \0 to these????
  if (strcmp(charBuf, "submerge") == 0) {
    strcpy(response, "ACK SUBMERGING");
    shouldSubmerge = true;
  }
  else if (strcmp(charBuf, "pump") == 0) {
    if (!motorIs(SUCK)) {
      strcpy(response, "ACK PUMPING");
      overrideState = PUMP;
    }
    else {
      strcpy(response, "NACK RETURN FIRST");
    }
  }
  else if (strcmp(charBuf, "suck") == 0) {
    if (!motorIs(PUMP)) {
      strcpy(response, "ACK SUCKING");
      overrideState = SUCK;
    }
    else {
      strcpy(response, "NACK RETURN FIRST");
    }
  }
  else if (strcmp(charBuf, "return") == 0) {
    strcpy(response, "ACK RETURNING TO SCHEDULE");
    stop();
    overrideState = NO_OVERRIDE;
  }
  else {
    strcpy(response, "NACK INVALID COMMAND");
  }

  Serial.println(response);
  rf95.send((uint8_t*) response, strlen(response));
  rf95.waitPacketSent();
  return shouldSubmerge;
}

void transmitPressurePacket() {
  serprintf("Sending pressure packet #%d with content {", profileNum);
  for (int p = 0; p < PKT_LEN; p++) {
    serprintf("%d, ", pressurePacket[p]);
  }
  Serial.println("}");

  serprintf("Sending time packet #%d with content {", profileNum);
  for (int p = 0; p < PKT_LEN; p++) {
    serprintf("%d, ", timePacket[p]);
  }
  Serial.println("}");

  pressurePacket[PKT_IDX_PROFILE] = profileNum;
  timePacket[PKT_IDX_PROFILE] = profileNum;
  
  rf95.send(pressurePacket, PKT_LEN);
  rf95.waitPacketSent();
  
  rf95.send(timePacket, PKT_LEN);
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

bool motorIs(int doingThis) {
  // doingThis should be PUMP or SUCK
  return overrideState == doingThis ||
         (overrideState == NO_OVERRIDE && SCHEDULE[currentStage][0] == doingThis);
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

  serprintf("RFM95 radio @%d MHz\n", (int) RF95_FREQ);
}

void initPressureSensor() {
  pressureSensor.init();
  
  pressureSensor.setModel(MS5837::MS5837_02BA);
  pressureSensor.setFluidDensity(997);  // kg/m^3

  serprintf("PRESSURE: %f\n", pressureSensor.pressure());
}
