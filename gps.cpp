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
  Serial3.begin(9600);

//  while (SerialGps.available() <= 0) {
//    Serial.println(F("No GPS data received: check wiring"));
//  }

  if (Serial3.available() > 0) {
    char c = Serial3.read();
    tinygps.encode(c);
//    Serial.println(c);
    if (tinygps.date.isValid()) {
      year = tinygps.date.year();
      month = tinygps.date.month();
      day = tinygps.date.day();
    }
  }
}

void Gps::readGps() {
  Serial3.begin(9600);
  while (Serial3.available()) {
    char c = Serial3.read();
    tinygps.encode(c);
//     Serial.println(c);

    if (tinygps.location.isValid()) {
      lon = tinygps.location.lng();
      lon = lon * 100000;
      lat = tinygps.location.lat();
      lat = lat * 100000;
      alt = tinygps.altitude.meters();
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
  }

  if (millis() > 5000 && tinygps.charsProcessed() < 10)
    Serial.println("No GPS data received: check wiring");
}
