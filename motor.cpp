/**
  @file motor.cpp
  @author Hiroyuki Yoshihara
  @date Created: 20180515
*/
#include "motor.h"

Motor::Motor(int pinMotorFin, int pinMotorRin) {
  int _pinMotorFin = pinMotorFin;
  int _pinMotorRin = pinMotorRin;
  pinMode(_pinMotorFin, OUTPUT);
  pinMode(_pinMotorRin, OUTPUT);
  digitalWrite(_pinMotorFin, LOW);
  digitalWrite(_pinMotorRin, LOW);
}

Motor::~Motor() {
}

void Motor::go(int v) {
  if (v < 0) v = 0;
  if (v > 255) v = 0;
  velocity = v;
  analogWrite(_pinMotorFin, velocity);
  digitalWrite(_pinMotorRin, LOW);
}

void Motor::back(int v) {
  if (v < 0) v = 0;
  if (v > 255) v = 0;
  analogWrite(_pinMotorRin, v);
  digitalWrite(_pinMotorFin, LOW);
}

void Motor::stop() {
  velocity = 0;
  digitalWrite(_pinMotorFin, LOW);
  digitalWrite(_pinMotorRin, LOW);
}

void Motor::stopSlowly() {
  if (velocity != 0) {
    for (int i = 0; i < 25; i++) {
      analogWrite(_pinMotorFin, velocity - 10 * i);
      digitalWrite(_pinMotorRin, LOW);
      delay(200);
    }
    velocity = 0;
  }
  digitalWrite(_pinMotorFin, LOW);
  digitalWrite(_pinMotorRin, LOW);
}

void Motor::brake() {
  velocity = 0;
  digitalWrite(_pinMotorFin, HIGH);
  digitalWrite(_pinMotorRin, HIGH);
}
