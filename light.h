/**
  @file light.h
  @author Hiroyuki Yoshihara
  @date Created: 20180415
*/
#ifndef _LIGHT_H_
#define _LIGHT_H_
#include <stdio.h>
#include <string.h>
#include "Arduino.h"

class Light {
  public:
  // 関数
  Light(int lightPin);
  ~Light();
  void readLight();

  // 変数
  int _lightPin;
  int lightValue;
};

#endif _LIGHT_H_

