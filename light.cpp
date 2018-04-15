/**
  @file light.cpp
  @author Hiroyuki Yoshihara
  @date Created: 20180415
*/
#include "light.h"

Light::Light(int lightPin) {
  _lightPin = lightPin;
  pinMode(_lightPin, INPUT);
}

Light::~Light() {
}

void Light::readLight() {
  lightValue = analogRead(_lightPin);
}
