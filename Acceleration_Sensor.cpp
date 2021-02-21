#include "Acceleration_Sensor.h"

//コンストラクタ
AccelSensor::AccelSensor(int _pin_x,int _pin_y,int _pin_z,int _count){
  pin_x=_pin_x;
  pin_y=_pin_y;
  pin_z=_pin_z;
  pinMode(pin_x,INPUT);
  pinMode(pin_y,INPUT);
  pinMode(pin_z,INPUT);
  count=_count;//(count)回比較して一致するなら着地
  xx=yy=zz=1000;//特に意味はないが初期値がないと動かないので仕方なく
}

void AccelSensor::Calibration(){
  xcal1=ycal1=zcal1=0;
  xcal=ycal=zcal=0.0;
  for(int i=0;i<100;i++){
  xcal1=xcal1+analogRead(pin_x);
  ycal1=ycal1+analogRead(pin_y);
  zcal1=zcal1+analogRead(pin_z);
  delay(10);
  }
  xcal=(float)xcal1/100.0;
  ycal=(float)ycal1/100.0;
  zcal=(float)zcal1/100.0;
  }

void AccelSensor::Accel_Read(){
  x1=y1=z1=0;
  for(int i=0;i<25;i++){
  x1=x1+analogRead(pin_x);
  y1=y1+analogRead(pin_y);
  z1=z1+analogRead(pin_z);
  delay(10);
  }
  x=(float)x1/25-xcal;
  y=(float)y1/25-ycal;
  z=(float)z1/25-zcal;
}

void AccelSensor::Landing_Detection(int _count){

//着地したらLEDを光らせる。動いている間はLEDを光らせない。
int band=5;
  if(-band<xx-x&&xx-x<band&&-band<yy-y&&yy-y<band&&-band<zz-z&&zz-z<band){
    count--;
    if(count<0){
      Serial.println("着地なう");
    }
  }else{
    xx=x;
    yy=y;
    zz=z;
    count=_count;
  }
  Serial.print(" X:");
  Serial.print(x);
  Serial.print(" Y:");
  Serial.print(y);
  Serial.print(" Z:");
  Serial.println(z);
  Serial.println(count);
}
//

