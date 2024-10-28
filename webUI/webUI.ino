#include <Wire.h>
#include <SPI.h>
#include <WiFiS3.h>
#include "blynkCredentials.h"
#include "wifiCredentials.h"
#include <BlynkSimpleWifi.h>
#include "config.h"

enum class Signals : uint8_t {
  SIGNAL_NOT_KNOWN,
  RED,
  YELLOW,
  DOUBLE_YELLOW,
  GREEN
};

Signals signalNodesState[4];

uint8_t axleCounter;
uint8_t  signalReset, homeSignalBtnState;

uint8_t i2cTxPayload[2], i2cRxPayload[IIC_SIGNAL_NODE_PAYLOAD_LENGTH];
unsigned long currentMillis, prevMillis;
BlynkTimer timer;

BLYNK_WRITE(V2) {
  signalReset = (uint8_t) param.asInt();
}

BLYNK_WRITE(V3) {
  homeSignalBtnState = (uint8_t) param.asInt();
}

void setup() {
  Wire.begin();
  // Wire.setClock(400000);
  Blynk.begin(BLYNK_AUTH_TOKEN, SSID, PASS);
  timer.setInterval(WEB_PAGE_UPDATE_INTERVAL, updateWebPage);
}

void loop() {
  Blynk.run();
  timer.run();

  currentMillis = millis();

  // transmit and receive i2c data every IIC_TRANSMIT_INTERVAL
  if (currentMillis - prevMillis >= IIC_TRANSMIT_INTERVAL) {

    // update TX payload
    i2cTxPayload[IIC_SIGNAL_RESET_INDEX] = signalReset;
    i2cTxPayload[IIC_HOME_SIGNAL_STATE_INDEX] = homeSignalBtnState;

    // send I2C tx payload
    Wire.beginTransmission(SIGNAL_NODE_IIC_ADDRESS);
    Wire.write(i2cTxPayload, 2);
    Wire.endTransmission();
    // request data from the home signal
    Wire.requestFrom(SIGNAL_NODE_IIC_ADDRESS, IIC_SIGNAL_NODE_PAYLOAD_LENGTH);
    uint8_t i = 0;

    // read data from the home signal
    while (Wire.available() && i < IIC_SIGNAL_NODE_PAYLOAD_LENGTH) {
      i2cRxPayload[i] = Wire.read();
      ++i;
    }

    // extract payload data if 8 bytes have been read
    if (i == IIC_SIGNAL_NODE_PAYLOAD_LENGTH) {
      extractIICPayload();
    }
    prevMillis = currentMillis;
  }
}

//  extract I2C rx payload into signalNodeState
void extractIICPayload() {
  uint8_t temp = 0;
  axleCounter = i2cRxPayload[IIC_AXLE_COUNT_INDEX];

  switch (i2cRxPayload[IIC_SOURCE_ID_INDEX]) {
    case FIRST_SIGNAL_NODE_ID:
      temp = i2cRxPayload[IIC_C_N1_NODE_INDEX];
      signalNodesState[1] = (Signals)(temp & 0x0F);
      signalNodesState[0] = (Signals)(temp >> 4);

      temp = i2cRxPayload[IIC_N2_N3_NODE_INDEX];
      signalNodesState[2] = (Signals)(temp >> 4);
      temp = i2cRxPayload[IIC_N2_N3_NODE_INDEX];
      signalNodesState[3] = (Signals)(temp & 0x0F);
      break;

    case LAST_SIGNAL_NODE_ID:
      temp = i2cRxPayload[IIC_C_N1_NODE_INDEX];
      signalNodesState[0] = (Signals)(temp >> 4);

      temp = i2cRxPayload[IIC_P2_P1_NODE_INDEX];
      signalNodesState[1] = (Signals)(temp & 0x0F);
      signalNodesState[2] = (Signals)(temp >> 4);

      temp = i2cRxPayload[IIC_STATUS_P3_NODE_INDEX];
      signalNodesState[3] = (Signals)(temp & 0x0F);
      break;

    default:
      break;
  }
}

// convert signal value into string
const char *signalStateToStr(Signals sig) {
  switch (sig) {
    case Signals::RED:
      return "R";
      break;

    case Signals::YELLOW:
      return "Y";
      break;

    case Signals::DOUBLE_YELLOW:
      return "DY";
      break;

    case Signals::GREEN:
      return "G";
      break;

    default:
      break;
  }
  return "X";
}

// updates the web page
void updateWebPage() {
  char signalStr[23];
  Blynk.virtualWrite(V0, axleCounter); // write axle counter value
  sprintf(signalStr, "| %s | %s | %s | %s |", signalStateToStr(signalNodesState[0]), signalStateToStr(signalNodesState[1]), signalStateToStr(signalNodesState[2]), signalStateToStr(signalNodesState[3]));
  Blynk.virtualWrite(V1, signalStr); // write signal state of signals to signal state label
}
