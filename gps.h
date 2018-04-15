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
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

class Gps {
  public:
    // 関数
    Gps(int gpsRx, int gpsTx);
    ~Gps();
    TinyGPSPlus tinygps;
    void setupGps();
    void readGps();

    // 変数
    int _gpsRx;
    int _gpsTx;
    float lon;   // 経度
    float lat;   // 緯度
    short year, month, day, hour, minute, second;
    float baudrate;
    float deg;
    float alt;
    String gpsDate; 
};

#endif
