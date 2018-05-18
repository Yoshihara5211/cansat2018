/**
  @file gps.cpp
  @author Hiroyuki Yoshihara
  @date Created: 20180415
*/
#include "gps.h"

Gps::Gps() {
}

Gps::~Gps() {
}

void Gps::setupGps() {
  SerialGps.begin(9600);
  while (SerialGps.available() > 0) {
    char c = SerialGps.read();
    tinygps.encode(c);
    if (tinygps.date.isValid()) {
      year = tinygps.date.year();
      month = tinygps.date.month();
      day = tinygps.date.day();
      String data1 = "year, month, day, hour, minute, second";
      String data2 = "lon, lat, alt, baudrate, deg";
      Serial.println(data1);
      Serial.println(data2);
    }
  }
}

void Gps::readGps() {
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
   
    gpsDate = String(year) + ", "
              + String(month) + ", "
              + String(day) + "，"
              + String(hour) + "，"
              + String(minute) + "，"
              + String(second) + "，"
              + String(lat) + ", "
              + String(alt) + "，"
              + String(baudrate) + "，"
              + String(deg) + "，"
              + String(alt) + "，";
    Serial.println(gpsDate);
  }
  if (millis() > 5000 && tinygps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}
