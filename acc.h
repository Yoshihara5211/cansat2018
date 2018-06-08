#ifndef _ACC_H_
#define _ACC_H_
#include <stdio.h>
#include <string.h>
#include "Arduino.h"

class Acc {
  public:
    // 関数
    Acc(int accxPin, int accyPin, int acczPin);
    ~Acc();
    void setupAcc();
    void readAcc();

    // 変数
    int _accxPin, _accyPin, _acczPin;
    unsigned long x1, y1, z1;
    int x, y, z;
    unsigned long xcal1, ycal1, zcal1;
    float xcal, ycal, zcal;
    int ax, ay, az;
};

#endif
