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
    HardwareSerial SerialRadio = Serial2;  // Serial2を使う
    void setupRadio();  // setup
    void getData();  // 地上局からステートが送られてきたら受け取る
    void sendData(String radio_data);  // データ送信

    // 変数  
    int radio_get_data=48;  
};

#endif
