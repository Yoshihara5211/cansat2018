/**
  @file light.cpp
  @author Hiroyuki Yoshihara
  @date Created: 20180415
*/
#include "light.h"

Light::Light(int lightPin) {
   _lightPin = lightPin;
  pinMode(_lightPin, INPUT);
//pinMode(A3, INPUT);
  lightValue = 0;
}

Light::~Light() {
}

void Light::readLight() {
  lightValue = analogRead(_lightPin);
//lightValue = analogRead(A3);
}

int Light::returnLightValue(){
  return lightValue;
}
