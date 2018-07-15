#include "acc.h"

Acc::Acc(int accxPin,int accyPin,int acczPin){
  _accxPin=accxPin;
  _accyPin=accyPin;
  _acczPin=acczPin;
  pinMode(_accxPin,INPUT);
  pinMode(_accyPin,INPUT);
  pinMode(_acczPin,INPUT);
}

Acc::~Acc(){
  }

void Acc::setupAcc(){
  xcal1=ycal1=zcal1=0;
  xcal=ycal=zcal=0.0;
  float count = 500;
  for(int i=0;i<count;i++){
  xcal1=xcal1+analogRead(_accxPin);
  ycal1=ycal1+analogRead(_accyPin);
  zcal1=zcal1+analogRead(_acczPin);
  delay(10);
  }
  xcal=(float)xcal1/count;
  ycal=(float)ycal1/count;
  zcal=(float)zcal1/count - 68.5 ;
  }

void Acc::readAcc(){
  x1=y1=z1=0;
  for(int i=0;i<25;i++){
  x1=x1+analogRead(_accxPin);
  y1=y1+analogRead(_accyPin);
  z1=z1+analogRead(_acczPin);
  }
  x=(float)x1/25-xcal;
  y=(float)y1/25-ycal;
  z=(float)z1/25-zcal;
  ax=9.8*x/68.5;
  ay=9.8*y/68.5;
  az=9.8*z/68.5;
}
