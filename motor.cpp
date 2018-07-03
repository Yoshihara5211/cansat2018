/**
  @file motor.cpp
  @author Hiroyuki Yoshihara
  @date Created: 20180515
*/
#include "motor.h"

Motor::Motor(int pinVref, int pinIn1, int pinIn2) {
   _pinVref = pinVref;
   _pinIn1 = pinIn1;
   _pinIn2 = pinIn2;
  pinMode(_pinVref, OUTPUT);
  pinMode(_pinIn1, OUTPUT);
   pinMode(_pinIn2, OUTPUT);
  digitalWrite(_pinVref, LOW);
  digitalWrite(_pinIn1, LOW);
  digitalWrite(_pinIn2  , LOW);
}

Motor::~Motor() {
}

void Motor::go(int v) {
  if (v < 0) v = 0;
  if (v > 255) v = 0;
  velocity = v;
  analogWrite(_pinVref, v);
  digitalWrite(_pinIn1, HIGH);
  digitalWrite(_pinIn2, LOW);
}

void Motor::back(int v) {
  if (v < 0) v = 0;
  if (v > 255) v = 0;
  velocity=-v;
  analogWrite(_pinVref, v);
  digitalWrite(_pinIn1, LOW);
  digitalWrite(_pinIn2, HIGH);
}

void Motor::stop() {
  velocity = 0;
  analogWrite(_pinVref, 0);
  digitalWrite(_pinIn1, LOW);
  digitalWrite(_pinIn2, LOW);
}

void Motor::stopSlowly() {
  if (velocity != 0) {
    for (int i = 0; i < 25; i++) {
      analogWrite(_pinVref, velocity - 10 * i);
       digitalWrite(_pinIn1, HIGH);
       digitalWrite(_pinIn2, LOW);
      delay(200);
    }
    velocity = 0;
  }
    analogWrite(_pinVref, 0);
  digitalWrite(_pinIn1, LOW);
  digitalWrite(_pinIn2, LOW);
}

void Motor::brake() {
  velocity = 0;
  analogWrite(_pinVref, 0);
  digitalWrite(_pinIn1, HIGH);
  digitalWrite(_pinIn2, HIGH);
}

