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


// Buffers for pressures & time data w/preambles containing:
// 1 byte team number, 1 byte profile index (0x00, 0x01), and 1 byte profile half index (0x00, 0x01)
// Every pressure is a 32-bit float stored as 4 byte entries
// Every time is a 32-bit unsigned long stored as 4 byte entries
// Times and pressures are interleaved: [team #, profile index, profile half, time, pressure, time, pressure, ...]
// 3 bytes preamble + (62 readings or times * 4 bytes = 248 bytes) = 251 bytes total = PKT_LEN = RH_RF95_MAX_MESSAGE_LEN
#define PKT_LEN              RH_RF95_MAX_MESSAGE_LEN
#define PKT_IDX_TEAM_NUM     0
#define PKT_IDX_PROFILE_NUM  1
#define PKT_IDX_PROFILE_HALF 2
#define PKT_PREAMBLE_LEN     3
#define PKT_PAYLOAD_LEN      PACKET_LEN - PACKET_PREAMBLE_LEN

#define RF95_FREQ 877.0


// Converts byte arrays to floats via shared memory
union {
  float floatVal;
  unsigned long longVal;
  byte byteArray[4];
} bytesUnion;

/**
 * printf the arguments with a line break at the end to the provided serial port.
 *  i.e. printfln(&Serial, "Here's a float: %f", 123.45);
 * Supports these interpolation codes:
 *  %% : escaped "%"
 *  %s : strings
 *  %d : integers (decimal)
 *  %b : integers (binary)
 *  %o : integers (octal)
 *  %x : integers (hex)
 *  %f : floats/doubles
*/
void serprintf(const char* input...);
