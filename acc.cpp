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
  for(int i=0;i<100;i++){
  xcal1=xcal1+analogRead(_accxPin);
  ycal1=ycal1+analogRead(_accyPin);
  zcal1=zcal1+analogRead(_acczPin);
  delay(10);
  }
  xcal=(float)xcal1/100.0;
  ycal=(float)ycal1/100.0;
  zcal=(float)zcal1/100.0 - 100.0 ;
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
  ax=100*x/110;
  ay=100*y/111;
  az=z;
}
