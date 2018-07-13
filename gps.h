/**
  @file gps.h
  @author Hiroyuki Yoshihara
  @date Created: 20180415
*/
#ifndef _GPS_H_
#define _GPS_H_
#include <stdio.h>
#include <string.h>
#include "Arduino.h"
#include <HardwareSerial.h>
#include <TinyGPS++.h>

class Gps {
  public:
    // 関数
    Gps();
    ~Gps();
   HardwareSerial SerialGps = Serial1;
    TinyGPSPlus tinygps;  // ライブラリからオブジェクト生成
    void setupGps();  // setup + date情報
    void readGps();  //  GPS情報読み取り

    // 変数
    short year, month, day, hour, minute, second;
    float lon;   // 経度[m]
    float lat;   // 緯度[m]
    float baudrate;  // 通信速度
    float deg;  // ??? 
    float alt;  // 高度(割と雑)
};

#endif
