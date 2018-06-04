
#ifndef _ACCELARATION_SENSOR_
#define _ACCELARATION_SENSOR_
#include "Arduino.h"

class AccelSensor{
  public:
  AccelSensor(int pin_x,int pin_y,int pin_z,int count);
  void Calibration();
  void Accel_Read();
  void Landing_Detection(int count);//着地検知
  float xx,yy,zz;//このxxとかにあるタイミングでのx,y,zの加速度を記録、以降のx,y,zの値と比較して変化がなければ着地検知
  int count;//xxとxが(count)回比較して変化がなければ着地
  unsigned long x1,y1,z1;
  int x,y,z;
  unsigned long xcal1,ycal1,zcal1;
  float xcal,ycal,zcal;

  private:
  int pin_x,pin_y,pin_z;
};

#endif

