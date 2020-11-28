/*
This library is based on the Arduino sketch by Nickduino: https://github.com/Nickduino/Somfy_Remote
*/

#include "Somfy_Remote.h"

#if defined __AVR_ATmega168__
#define EEPROM_SIZE 512
#elif defined __AVR_ATmega328__
#define EEPROM_SIZE 1024
#elif defined __AVR_ATmega2560__
#define EEPROM_SIZE 4096
#elif ESP32 || ESP8266
#define EEPROM_SIZE 512
#endif

const uint16_t symbol = 604;

uint8_t currentEppromAddress = 0;
uint8_t gdo0Pin;
uint8_t gdo2Pin;

// Constructor
SomfyRemote::SomfyRemote(String name, uint32_t remoteCode)
{
  _name = name;
  _remoteCode = remoteCode;
  _eepromAddress = getNextEepromAddress();
}

// Getter for name
String SomfyRemote::getName()
{
  return _name;
}

// Generates the next available EEPROM address
uint16_t SomfyRemote::getNextEepromAddress()
{
  uint8_t eppromAddress = currentEppromAddress;

  // Every address gets 4 bytes of space to save the rolling code0
  currentEppromAddress = currentEppromAddress + 4;

  return eppromAddress;
}

// Reads the current rolling code
void SomfyRemote::getRollingCode()
{
  // Set new rolling code if not already set
  if (EEPROM.get(_eepromAddress, _rollingCode) < 1)
  {
    _rollingCode = 1;
  }
}

// Send a command to the blinds
void SomfyRemote::move(String command)
{
  const uint8_t up = 0x2;
  const uint8_t down = 0x4;
  const uint8_t my = 0x1;
  const uint8_t prog = 0x8;

  uint8_t frame[7];

  EEPROM.begin(EEPROM_SIZE);

  getRollingCode();

  // Build frame according to selected command
  command.toUpperCase();

  switch (command[0])
  {
  case 'U':
    buildFrame(frame, up);
    break;
  case 'D':
    buildFrame(frame, down);
    break;
  case 'M':
    buildFrame(frame, my);
    break;
  case 'P':
    buildFrame(frame, prog);
    break;
  default:
    buildFrame(frame, my);
    break;
  }

  // Send the frame according to Somfy RTS protocol
  sendCommand(frame, 2);
  for (int i = 0; i < 2; i = i + 1)
  {
    sendCommand(frame, 7);
  }

  EEPROM.commit();
}

// Build frame according to Somfy RTS protocol
void SomfyRemote::buildFrame(uint8_t *frame, uint8_t command)
{
  uint8_t checksum = 0;

  frame[0] = 0xA7;              // Encryption key.
  frame[1] = command << 4;      // Selected command. The 4 LSB are the checksum
  frame[2] = _rollingCode >> 8; // Rolling code (big endian)
  frame[3] = _rollingCode;      // Rolling code
  frame[4] = _remoteCode;       // Remote address
  frame[5] = _remoteCode >> 8;  // Remote address
  frame[6] = _remoteCode >> 16; // Remote address

  // Checksum calculation (XOR of all nibbles)
  for (uint8_t i = 0; i < 7; i = i + 1)
  {
    checksum = checksum ^ frame[i] ^ (frame[i] >> 4);
  }
  checksum &= 0b1111; // Keep the last 4 bits only

  //Checksum integration
  frame[1] |= checksum; //  If a XOR of all the nibbles is equal to 0, the blinds will
                        // consider the checksum ok.

  // Obfuscation (XOR of all bytes)
  for (uint8_t i = 1; i < 7; i = i + 1)
  {
    frame[i] ^= frame[i - 1];
  }

  _rollingCode = _rollingCode + 1;

  EEPROM.put(_eepromAddress, _rollingCode); //  Store the new value of the rolling code in the EEPROM.
}

// Send frame according to Somfy RTS protocol
void SomfyRemote::sendCommand(uint8_t *frame, uint8_t sync)
{
  if (sync == 2)
  { // Only with the first frame.

// Set pins according to module
#if defined __AVR_ATmega168__ || defined __AVR_ATmega328__ || defined __AVR_ATmega2560__
    gdo0Pin = 2;
    gdo2Pin = 3;
#elif ESP32 || ESP8266
    gdo0Pin = 2;
    gdo2Pin = 4;
#endif

    ELECHOUSE_cc1101.setGDO(gdo0Pin, gdo2Pin);

    // Initialize radio chip
    ELECHOUSE_cc1101.Init();

    // Enable transmission at 433.42 MHz
    ELECHOUSE_cc1101.SetTx(433.42);

    // Wake-up pulse & Silence
    digitalWrite(gdo2Pin, HIGH); // High
    delayMicroseconds(9415);
    digitalWrite(gdo2Pin, LOW); // Low
    delayMicroseconds(89565);
  }

  // Hardware sync: two sync for the first frame, seven for the following ones.
  for (int i = 0; i < sync; i = i + 1)
  {
    digitalWrite(gdo2Pin, HIGH); // PIN 2 HIGH
    delayMicroseconds(4 * symbol);
    digitalWrite(gdo2Pin, LOW); // PIN 2 LOW
    delayMicroseconds(4 * symbol);
  }

  // Software sync
  digitalWrite(gdo2Pin, HIGH); // PIN 2 HIGH
  delayMicroseconds(4550);
  digitalWrite(gdo2Pin, LOW); // PIN 2 LOW
  delayMicroseconds(symbol);

  //Data: bits are sent one by one, starting with the MSB.
  for (uint8_t i = 0; i < 56; i = i + 1)
  {
    if (((frame[i / 8] >> (7 - (i % 8))) & 1) == 1)
    {
      sendBit(true);
    }
    else
    {
      sendBit(false);
    }
  }
  digitalWrite(gdo2Pin, LOW); // PIN 2 LOW
  delayMicroseconds(30415);   // Inter-frame silence
}

// Send one bit
void SomfyRemote::sendBit(bool value)
{
  uint8_t firstState;
  uint8_t secondState;

  // Decide which bit to send (Somfy RTS bits are manchester encoded: 0 = high->low 1 = low->high)
  if (value == true)
  {
    firstState = LOW;
    secondState = HIGH;
  }
  else if (value == false)
  {
    firstState = HIGH;
    secondState = LOW;
  }

  // Send the bit
  digitalWrite(gdo2Pin, firstState);
  delayMicroseconds(symbol);
  digitalWrite(gdo2Pin, secondState);
  delayMicroseconds(symbol);
}
