// FLOAT TRANSCEIVER sketch originally built from:
//  rf95_client.pde
//  -*- mode: C++ -*-
//
// REQUIRED LIBRARIES:
// RadioHead v1.122.1 by Mike McCauley
// Blue Robotics MS5837 Library v1.1.1 by BlueRobotics
//
// Example sketch showing how to create a simple messageing client
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf95_server
// Tested with Anarduino MiniWirelessLoRa, Rocket Scream Mini Ultra Pro with
// the RFM95W, Adafruit Feather M0 with RFM95

#include <SPI.h>
#include <RH_RF95.h>
#include "MS5837.h"

//#ifdef __arm__
//// should use uinstd.h to define sbrk but Due causes a conflict
//extern "C" char* sbrk(int incr);
//#else  // __ARM__
//extern char *__brkval;
//#endif  // __arm__
//
//int freeMemory() {
//  char top;
//#ifdef __arm__
//  return &top - reinterpret_cast<char*>(sbrk(0));
//#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
//  return &top - __brkval;
//#else  // __arm__
//  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
//#endif  // __arm__
//}

int freeMemory() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0  ? (int)&__heap_start : (int) __brkval);  
}


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

#define PACKET_PREAMBLE_LEN 1
#define PACKET_PAYLOAD_LEN RH_RF95_MAX_MESSAGE_LEN - PACKET_PREAMBLE_LEN

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

uint8_t SCHEDULE_LENGTH = 11;
unsigned long SCHEDULE[][2] = {
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

unsigned long previousTime;
unsigned long previousPressureReadTime;


/************ Radio Setup ***************/

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


// Singleton instance of the pressure sensor driver
MS5837 pressureSensor;

// Buffer of pressures w/max length for 1 profile w/readings every PRESSURE_READ_INTERVAL
// Must divide and cast before adding to avoid overflowing while keeping this a legal index
const int PRESSURE_BUFFER_LENGTH = sizeof(float) * (
  (int) (SUCK_MAX / PRESSURE_READ_INTERVAL) +
  (int) (DESCEND_TIME / PRESSURE_READ_INTERVAL) +
  (int) (PUMP_MAX / PRESSURE_READ_INTERVAL) + 
  (int) (ASCEND_TIME / PRESSURE_READ_INTERVAL)
);
uint8_t pressureBuffer[PRESSURE_BUFFER_LENGTH];
int pressureBufferIndex;
// Converts floats to byte arrays via shared memory
union {
  float floatVar;
  uint8_t byteArray[4];
} floatBytesUnion;

void setup() {
  Serial.begin(115200);
  // Wait until serial console is open; remove if not tethered to computer
  while (!Serial) ;
  
  Serial.println("Float Transceiver");
  Serial.println();

  previousTime = millis();
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

  initRadio();
  initPressureSensor();
}


void loop() {

  // Move to next stage if necessary
  bool submergeReceived = signalReceived();

  // Read the pressure if we're profiling
  if (
    ((1 <= stage && stage <= 4) || (6 <= stage && stage <= 9)) &&
    millis() >= previousPressureReadTime + PRESSURE_READ_INTERVAL &&
    pressureBufferIndex < PRESSURE_BUFFER_LENGTH  // Stop recording if we overflow buffer
  ) {
    Serial.print("Reading: ");
    previousPressureReadTime = millis();
    floatBytesUnion.floatVar = 5.0; // pressureSensor.pressure();
    memcpy(pressureBuffer + pressureBufferIndex, floatBytesUnion.byteArray, sizeof(float));
    Serial.print(pressureBuffer[pressureBufferIndex]);
    Serial.print(", ");
    Serial.print(pressureBuffer[pressureBufferIndex + 1]);
    Serial.print(", ");
    Serial.print(pressureBuffer[pressureBufferIndex + 2]);
    Serial.print(", ");
    Serial.println(pressureBuffer[pressureBufferIndex + 3]);
    pressureBufferIndex += sizeof(float);
    Serial.print(" >> ");
    Serial.println(freeMemory());
  }

  // Transmit the pressure buffer if we're surfaced
  if (stage == 5 || stage == 10) {
    transmitPressureBuffer();
  }

  if (
    millis() >= previousTime + SCHEDULE[stage][1] ||
    (SCHEDULE[stage][0] == WAIT && submergeReceived) ||
    (SCHEDULE[stage][0] == SUCK && !digitalRead(LIMIT_FULL)) ||
    (SCHEDULE[stage][0] == PUMP && !digitalRead(LIMIT_EMPTY))
  ) {
    Serial.print(stage);
    Serial.print(" ");
    Serial.print(SCHEDULE[stage][0]);
    Serial.print(" ");
    Serial.print(previousTime);
    Serial.print(" ");
    Serial.print(SCHEDULE[stage][1]);
    Serial.print(" ");
    Serial.print(millis());
    Serial.print(" ");
    Serial.print(millis() >= previousTime + SCHEDULE[stage][1]);
    Serial.println();

    // Reset the pressure buffer index if we just finished our time surfaced
    if (stage == 5 || stage == 10) {
      pressureBufferIndex = 0;
    }

    previousTime = millis();
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

bool signalReceived() {
  if (rf95.available()) {
    Serial.println("RF has signal");
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    Serial.println(len);
    if (rf95.recv(buf, &len)) {
      if (!len) return false;
      buf[len] = 0;
      Serial.print("Received [");
      Serial.print(len);
      Serial.print("]: '");
      Serial.print((char*) buf);
      Serial.println("'");
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);

      Serial.print(buf[0]);
      Serial.print(", ");
      Serial.println(buf[1]);

      if (strcmp((char*) buf, "su") == 0) {
        uint8_t test_packet[] = {50, 60};
        rf95.send(test_packet, strlen(test_packet));
        rf95.waitPacketSent();
        return true;
      }
      else {
        Serial.println("Invalid command");
      }
    }
    else {
      Serial.println("Receive failed");
    }
  }

  return false;
}

void transmitPressureBuffer() {
  uint8_t packetNum = 0;
  int packetStart = 0;
  int fullMessageLength =
    pressureBufferIndex < PRESSURE_BUFFER_LENGTH ? pressureBufferIndex : PRESSURE_BUFFER_LENGTH;
  while (packetStart < fullMessageLength) {
    int packetLength = fullMessageLength - packetStart;  // Remaining bytes in message
    if (packetLength > PACKET_PAYLOAD_LEN) {
      packetLength = PACKET_PAYLOAD_LEN;
    }
    
    uint8_t packet[PACKET_PREAMBLE_LEN + packetLength];
    packet[0] = packetNum;
    for (int i = 0; i < packetLength; i++) {
      packet[i + PACKET_PREAMBLE_LEN] = pressureBuffer[packetStart + i];
    }

    Serial.print(" >> ");
    Serial.println(freeMemory());

    Serial.print("Sending packet #");
    Serial.print(packetNum);
    Serial.print(" at index ");
    Serial.print(packetStart);
    Serial.print(" with preamble length ");
    Serial.print(PACKET_PREAMBLE_LEN);
    Serial.print(", payload length ");
    Serial.print(packetLength);
    Serial.print(", and content {");
    for (int p = 0; p < packetLength; p++) {
      Serial.print(packet[p]);
      Serial.print(", ");
    }
    Serial.println("}");

    packetNum++;
    packetStart += packetLength;

    Serial.print("Sizes: ");
    Serial.print(RH_RF95_MAX_MESSAGE_LEN);
    Serial.print(" ");
    Serial.print(PACKET_PREAMBLE_LEN);
    Serial.print(" ");
    Serial.print(PACKET_PAYLOAD_LEN);
    Serial.print(" ");
    Serial.print(packetLength);
    Serial.println();
    
    rf95.send(packet, packetLength);
    Serial.print(" >> ");
    Serial.println(freeMemory());
    rf95.waitPacketSent();
    delay(1000);
  }
}


void pump() {
  digitalWrite(MOTOR_PWM, HIGH);
  digitalWrite(PUMP_PIN, HIGH);
  digitalWrite(SUCK_PIN, LOW);
}

void suck() {
  digitalWrite(MOTOR_PWM, HIGH);
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(SUCK_PIN, HIGH);
}

void stop() {
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
  pressureSensor.setFluidDensity(1000);  // kg/m^3
}
