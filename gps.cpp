/**
  @file gps.cpp
  @author Hiroyuki Yoshihara
  @date Created: 20180415
*/
#include "gps.h"

Gps::Gps(int gpsRx, int gpsTx) {
  _gpsRx = gpsRx;
  _gpsTx = gpsTx;
}

Gps::~Gps() {
}

void Gps::setupGps() {
  SoftwareSerial SerialGps(_gpsRx, _gpsTx);
  SerialGps.begin(9600);
  while (SerialGps.available() > 0) {
    char c = SerialGps.read();
    tinygps.encode(c);
    if (tinygps.date.isValid()) {
      year = tinygps.date.year();
      month = tinygps.date.month();
      day = tinygps.date.day();
    }
  }
}

void Gps::readGps() {
  SoftwareSerial SerialGps(_gpsRx, _gpsTx);
  SerialGps.begin(9600);
  while (SerialGps.available()) {
    char c = SerialGps.read();
    tinygps.encode(c);

    if (tinygps.location.isValid()) {
      lon = tinygps.location.lng();
      lat = tinygps.location.lat();
    }

    if (tinygps.time.isValid()) {
      hour = tinygps.time.hour();
      minute = tinygps.time.minute();
      second = tinygps.time.second();
    }

    if (tinygps.speed.isValid()) {
      baudrate = tinygps.speed.mps();
    }

    if (tinygps.course.isValid()) {
      deg = tinygps.course.deg();
    }
    alt = tinygps.altitude.meters();
gpsDate = String(lon) + ", "
            + String(lat) + ", "
            + String(alt);
    Serial.println(gpsDate);
  }
   if (millis() > 5000 && tinygps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}
