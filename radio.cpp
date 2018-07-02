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
  Serial2.begin(9600);
  String radio_name = "state, millis, light, lat, lon, ax, ay, az, deg, mic... ";
  Serial2.println(radio_name);
}

void Radio::getData() {
  if (SerialRadio.available()) {
    radio_get_data = SerialRadio.read();
  }
}

void Radio::sendData(String radio_data) {
  SerialRadio.end();
  SerialRadio.begin(9600);
  String _radio_data = radio_data;
  Serial2.println(_radio_data);
  Serial.println(_radio_data);
}





