/**
  @file sd.cpp
  @author Hiroyuki Yoshihara
  @date Created: 20180501
*/
#include "sd.h"

Sd::Sd() {
}

Sd::~Sd() {
}

void Sd::setupSd() {
  pinMode(SS, OUTPUT);
  // SSピン（Unoは10番、Megaは53番）は使わない場合でも出力にする
  // そうしないと、SPIがスレーブモードに移行し、SDライブラリが動作しなくなる
  Serial.print("SD read write test start...\n");
  Serial.print("Initializing SD card...");          //check the Serial communication
  if (!SD.begin()) {  // check the SD card is available or not,
    Serial.println("Card failed, or not present");  // in the case of SD card error,
  }
  else {
    Serial.println("Card initialized.");              //in the case of SD card is available,
  }
  dataSet = "time, lon, lat, x, y, z";
  printSd(dataSet);
}

void Sd::printSd(String _dataIn) {
  File file = SD.open("cansat.txt", FILE_WRITE);
  dataIn = _dataIn;
  if (file) {                                      //if the file in the SD card was open to wrihte, true or false
    file.println(dataIn);                          // write data into the file,
    file.close();                                  // close the file,
    Serial.println(dataIn);                           // print to the serial port too,
  }
  else {                                              // if the file isn't open, pop up an error message,
    Serial.println("error opening file");
  }
}
