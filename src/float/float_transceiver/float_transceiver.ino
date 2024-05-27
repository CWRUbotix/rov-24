// FLOAT TRANSCEIVER sketch originally built from:
//  rf95_client.pde
//  -*- mode: C++ -*-
//
// REQUIRED LIBRARIES:
// RadioHead v1.122.1 by Mike McCauley
// Blue Robotics MS5837 Library v1.1.1 by BlueRobotics

#include <RH_RF95.h>
#include <SPI.h>

#include "MS5837.h"
#include "rov_common.hpp"

// H-bridge direction control pins
#define MOTOR_PWM 6   // Leave 100% cycle for top speed
#define PUMP_PIN  9   // Set high for pump (CW when facing down)
#define SUCK_PIN  10  // Set high for suck (CCW when facing down)

// Limit switch pins
#define LIMIT_FULL  12  // Low when syringe is full
#define LIMIT_EMPTY 11  // Low when syringe is empty

#define TEAM_NUM               25
#define PRESSURE_READ_INTERVAL 200
#define PACKET_SEND_INTERVAL   1000
#define FLOAT_PKT_RX_TIMEOUT   900

// Schedule (all delays in ms)
#define RELEASE_MAX  300000
#define SUCK_MAX     10000
#define DESCEND_TIME 10000
#define PUMP_MAX     10000
#define ASCEND_TIME  10000
#define TX_MAX       60000
#define ONE_HOUR     360000

#define SCHEDULE_LENGTH 11

#define SEND_DEBUG_PACKETS false

enum class StageType { WaitDeploying, WaitTransmitting, WaitProfiling, Suck, Pump };
enum class OverrideState { NoOverride, Stop, Suck, Pump };
enum class MotorState { Stop, Suck, Pump };

struct Stage {
  StageType type;
  uint32_t timeoutMillis;
};

OverrideState overrideState = OverrideState::NoOverride;
uint8_t currentStage = 0;

Stage SCHEDULE[SCHEDULE_LENGTH] = {
  // Wait for max <time> or until surface signal
  {StageType::WaitDeploying,    RELEASE_MAX },

  // Profile 1
  {StageType::Suck,             SUCK_MAX    },
  {StageType::WaitProfiling,    DESCEND_TIME},
  {StageType::Pump,             PUMP_MAX    },
  {StageType::WaitProfiling,    ASCEND_TIME },

  {StageType::WaitTransmitting, TX_MAX      },

  // Profile 2
  {StageType::Suck,             SUCK_MAX    },
  {StageType::WaitProfiling,    DESCEND_TIME},
  {StageType::Pump,             PUMP_MAX    },
  {StageType::WaitProfiling,    ASCEND_TIME },

  {StageType::WaitTransmitting, ONE_HOUR    },
};

uint32_t stageStartTime;
uint32_t previousPressureReadTime;
uint32_t previousPacketSendTime;

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Singleton instance of the pressure sensor driver
MS5837 pressureSensor;

byte packets[2][PKT_LEN];
int packetIndex = PKT_HEADER_LEN;

uint8_t profileNum = 0;
uint8_t profileHalf = 0;

void setup() {
  Serial.begin(115200);
  // Wait until serial console is open; remove if not tethered to computer
  // while (!Serial) {}

  Serial.println("Float Transceiver");
  Serial.println();

  stageStartTime = millis();
  previousPressureReadTime = millis();
  previousPacketSendTime = millis();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(LIMIT_EMPTY, INPUT_PULLUP);
  pinMode(LIMIT_FULL, INPUT_PULLUP);

  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(SUCK_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);

  digitalWrite(MOTOR_PWM, LOW);
  digitalWrite(SUCK_PIN, LOW);
  digitalWrite(PUMP_PIN, LOW);

  clearPacketPayloads();

  packets[0][PKT_IDX_TEAM_NUM] = TEAM_NUM;
  packets[0][PKT_IDX_PROFILE_HALF] = 0;

  packets[1][PKT_IDX_TEAM_NUM] = TEAM_NUM;
  packets[1][PKT_IDX_PROFILE_HALF] = 1;

  initRadio();
  initPressureSensor();
}

void loop() {
  bool submergeReceived = receiveCommand();

  if (overrideState == OverrideState::Suck) {
    if (digitalRead(LIMIT_FULL) == HIGH) {
      suck();
    }
    else {
      stop();
      overrideState = OverrideState::Stop;
    }
    return;
  }
  else if (overrideState == OverrideState::Pump) {
    if (digitalRead(LIMIT_EMPTY) == HIGH) {
      pump();
    }
    else {
      stop();
      overrideState = OverrideState::Stop;
    }
    return;
  }
  else if (overrideState == OverrideState::Stop) {
    stop();
    return;
  }

  // Send tiny packet for judges while deploying
  if (stageIs(StageType::WaitDeploying) && millis() >= previousPressureReadTime + PRESSURE_READ_INTERVAL) {
    previousPressureReadTime = millis();
    pressureSensor.read();
    float pressure = pressureSensor.pressure();
    serialPrintf("Reading pressure at surface: %f\n", pressure);

    int intComponent = pressure;
    int fracComponent = trunc((pressure - intComponent) * 10000);
    char judgePacketBuffer[25];
    snprintf(judgePacketBuffer, 25, "%d,%lu,%d.%04d\0", TEAM_NUM, previousPressureReadTime, intComponent, fracComponent);
    Serial.println(judgePacketBuffer);

    rf95.send(judgePacketBuffer, strlen(judgePacketBuffer));
    rf95.waitPacketSent();
  }

  // Read the pressure if we're profiling
  if (
    !isSurfaced() && millis() >= previousPressureReadTime + PRESSURE_READ_INTERVAL &&
    packetIndex < PKT_LEN  // Stop recording if we overflow buffer
  ) {
    previousPressureReadTime = millis();
    memcpy(packets[profileHalf] + packetIndex, &previousPressureReadTime, sizeof(uint32_t));
    packetIndex += sizeof(uint32_t);

    pressureSensor.read();
    float pressure = pressureSensor.pressure();
    serialPrintf("Reading pressure: %f\n", pressure);
    memcpy(packets[profileHalf] + packetIndex, &pressure, sizeof(float));
    packetIndex += sizeof(float);

    if (profileHalf == 0 && packetIndex >= PKT_LEN) {
      profileHalf = 1;
      packetIndex = PKT_HEADER_LEN;
    }
  }

  #ifdef DO_DEBUGGING
  // Transmit the pressure buffer whenever we're surfaced
  bool isSurfacedToTransmit = isSurfaced();
  #else
  // Transmit the pressure buffer if we're surfaced after completing a profile
  bool isSurfacedToTransmit = stageIs(StageType::WaitTransmitting);
  #endif  // DO_DEBUGGING

  if (isSurfacedToTransmit && millis() >= previousPacketSendTime + PACKET_SEND_INTERVAL) {
    transmitPressurePacket();
    previousPacketSendTime = millis();
  }

  // Go to next stage if we've timed out or reached another stage end condition
  bool stageTimedOut = millis() >= stageStartTime + SCHEDULE[currentStage].timeoutMillis;
  bool suckingHitLimitSwitch = stageIs(StageType::Suck) && !digitalRead(LIMIT_FULL);
  bool pumpingHitLimitSwitch = stageIs(StageType::Pump) && !digitalRead(LIMIT_EMPTY);
  bool shouldSubmerge = isSurfaced() && submergeReceived;

  if (shouldSubmerge || stageTimedOut || suckingHitLimitSwitch || pumpingHitLimitSwitch) {
    serialPrintf(
      "Ending stage #%d, type: %d, start time: %l, max wait time: %l, current time: %l\n",
      currentStage, SCHEDULE[currentStage].type, stageStartTime,
      SCHEDULE[currentStage].timeoutMillis, millis());

    // If we just finished transmitting on the surface, go to the next profile
    if (stageIs(StageType::WaitTransmitting)) {
      packetIndex = PKT_HEADER_LEN;
      profileNum++;
      profileHalf = 0;
      clearPacketPayloads();
    }

    // === MOVE TO NEXT STAGE ===
    stageStartTime = millis();
    currentStage++;
    stop();
    // ==========================

    // If we signal a third profile, restart the schedule
    if (currentStage >= SCHEDULE_LENGTH) {
      currentStage = 1;
    }

    // Switch to sucking/pumping depending on the stage we're entering
    if (stageIs(StageType::Suck)) {
      suck();
    }
    else if (stageIs(StageType::Pump)) {
      pump();
    }
  }
}

bool receiveCommand() {
  Serial.println("Receiving command...");

  if (!rf95.waitAvailableTimeout(FLOAT_PKT_RX_TIMEOUT)) {
    return false;
  }

  Serial.println("RF has signal");
  byte len = RH_RF95_MAX_MESSAGE_LEN;
  byte byteBuffer[len];

  if (!rf95.recv(byteBuffer, &len)) {
    Serial.println("Receive failed");
    return false;
  }
  if (!len) {
    Serial.println("Received with length 0; dropping");
    return false;
  }

  byteBuffer[len] = 0;
  char* charBuf = reinterpret_cast<char*>(byteBuffer);

  serialPrintf("Received [%d]: '%s'\n", len, charBuf);

  String response;
  bool shouldSubmerge = false;

  if (strcmp(charBuf, "submerge") == 0) {
    response = "ACK SUBMERGING";
    shouldSubmerge = true;
  }
  else if (strcmp(charBuf, "pump") == 0) {
    if (getMotorState() != MotorState::Suck) {
      response = "ACK PUMPING";
      overrideState = OverrideState::Pump;
    }
    else {
      response = "NACK RETURN FIRST";
    }
  }
  else if (strcmp(charBuf, "suck") == 0) {
    if (getMotorState() != MotorState::Pump) {
      response = "ACK SUCKING";
      overrideState = OverrideState::Suck;
    }
    else {
      response = "NACK RETURN FIRST";
    }
  }
  else if (strcmp(charBuf, "stop") == 0) {
    response = "ACK STOPPING";
    overrideState = OverrideState::Stop;
  }
  else if (strcmp(charBuf, "return") == 0) {
    response = "ACK RETURNING TO SCHEDULE";
    stop();
    overrideState = OverrideState::NoOverride;
  }
  else {
    response = "NACK INVALID COMMAND";
  }

  byte responseBytes[response.length() + 1];
  response.getBytes(responseBytes, response.length() + 1);
  rf95.send(responseBytes, response.length() + 1);
  rf95.waitPacketSent();
  return shouldSubmerge;
}

void transmitPressurePacket() {
  for (int half = 0; half < 2; half++) {
    serialPrintf("Sending packet #%d half %d with content {", profileNum, half);
    for (int p = 0; p < PKT_LEN; p++) {
      serialPrintf("%d, ", packets[half][p]);
    }
    Serial.println("}");

    packets[half][PKT_IDX_PROFILE_NUM] = profileNum;

    rf95.send(packets[half], PKT_LEN);
    rf95.waitPacketSent();
  }
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

MotorState getMotorState() {
  if (overrideState != OverrideState::NoOverride) {
    switch (overrideState) {
      case OverrideState::Stop: return MotorState::Stop;
      case OverrideState::Suck: return MotorState::Suck;
      case OverrideState::Pump: return MotorState::Pump;
    }
  }

  switch (SCHEDULE[currentStage].type) {
    case StageType::WaitDeploying:
    case StageType::WaitTransmitting:
    case StageType::WaitProfiling: return MotorState::Stop;
    case StageType::Suck: return MotorState::Suck;
    case StageType::Pump: return MotorState::Pump;
  }
}

bool stageIs(StageType type) {
  return SCHEDULE[currentStage].type == type;
}

bool isSurfaced() {
  return stageIs(StageType::WaitDeploying) ||
         stageIs(StageType::WaitTransmitting);
}

void clearPacketPayloads() {
  for (int half = 0; half < 2; half++) {
    for (int i = PKT_HEADER_LEN; i < PKT_LEN; i++) {
      packets[half][i] = 0;
    }
  }
}

/******* Setup Methods (down here cause they're large) *******/

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
    while (1) {}
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

  serialPrintf("RFM95 radio @%d MHz\n", static_cast<int>(RF95_FREQ));
}

void initPressureSensor() {
  pressureSensor.init();

  pressureSensor.setModel(MS5837::MS5837_02BA);
  pressureSensor.setFluidDensity(997);  // kg/m^3

  serialPrintf("PRESSURE: %f\n", pressureSensor.pressure());
}
