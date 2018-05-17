/**
  @file xbee.h
  @author Hiroyuki Yoshihara
  @date Created: 20180512
*/
#ifndef _RADIO_H_
#define _RADIO_H_
#include <stdio.h>
#include <string.h>
#include "Arduino.h"
#include <HardwareSerial.h>
#include <XBee.h>

class Radio {
  public:
    // 関数
    Radio();
    ~Radio();
    XBee xbee = XBee();
    XBeeResponse response = XBeeResponse();
    ZBRxResponse rx = ZBRxResponse();
    HardwareSerial SerialXbee = Serial2;
    void setupRadio();
    void getData();
    void sendData(String s);

    // 変数
    String receiveString;
    int intData;
    int stateData;
};

#endif
