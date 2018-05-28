/**
  @file motor.h
  @author Hiroyuki Yoshihara
  @date Created: 20180515
*/
#ifndef _MOTOR_H_
#define _MOTOR_H_
#include <stdio.h>
#include <string.h>
#include "Arduino.h"

class Motor {
public:
  Motor(int pinMotorFin, int pinMotorRin);  // モータドライバのFIN,RIN番号
  ~Motor();
  void go(int v);  // 前進，vは0~255で0~5v
  void back(int v);  // 後進
  void stop();  //ストップ(電圧止める)
  void stopSlowly();  //徐々に減速してストップ(徐々に電圧弱める)
  void brake();  //ブレーキ(電圧かけてモーター固定)

  int velocity;  // 前進中の速度
  int _pinMotorFin;
  int _pinMotorRin;
};

#endif