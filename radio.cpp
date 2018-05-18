/**
  @file xbee.cpp
  @author Hiroyuki Yoshihara
  @date Created: 20180512
*/
#include "radio.h"
Radio::Radio() {
}

Radio::~Radio() {
}

void Radio::setupRadio() {
  SerialXbee.begin(9600);
}

void Radio::getData() {
  xbee.readPacket();
  if (xbee.getResponse().isAvailable()) {
    if (xbee.getResponse().getApiId() == ZB_EXPLICIT_RX_RESPONSE) {
      xbee.getResponse().getZBRxResponse(rx);

      if (xbee.getResponse().getFrameData()[4] == 0x41 && xbee.getResponse().getFrameData()[5] == 0x05) {
        for (int i = 0; i < rx.getDataLength(); i++) {
          receiveString = String(rx.getData()[i], DEC);
          intData = receiveString.toInt();
          if (intData <= 57 && intData >= 48) {
            stateData = intData - 48;
          }
        }
      }
    }
  }
}

void Radio::sendData(String s) {
  int len = s.length();
  uint8_t sendData[len];

  for (int i = 0; i < len; i++) {
    String tempstring = s.substring(i, i + 1);
    if (tempstring == ",") {
      sendData[i] = 44;
    } else if (tempstring == "-") {
      sendData[i] = 45;
    } else if (tempstring == ' ') {
      sendData[i] = 32;
    } else if (tempstring == '.') {
      sendData[i] = 46;
    } else {
      sendData[i] = tempstring.toInt() + 48;
    }
  }


  XBeeAddress64 addr64 = XBeeAddress64(0x00000000, 0x00000000);
  ZBTxRequest zbTx = ZBTxRequest(addr64, sendData, sizeof(sendData));
  xbee.send(zbTx);
  Serial.println("Sent to XBee!");
}
