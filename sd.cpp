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
  Serial.println("SD read write test start...");
  Serial.print("Initializing SD card..........");          //check the Serial communication
  if (!SD.begin()) {  // check the SD card is available or not,
    Serial.println("Card failed, or not present");  // in the case of SD card error,
  }
  else {
    Serial.println("Card initialized.");              //in the case of SD card is available,
  }
  
  String log_name = "state, millis, light, lat, lon, ax, ay, az, deg, mic... ";  // ログ保存データ名
  printSd(log_name);
}

void Sd::printSd(String log_data) {
  File file = SD.open("cansat.txt", FILE_WRITE);
  String _log_data = log_data;  // ログ保存データ
  if (file) {                                      //if the file in the SD card was open to wrihte, true or false
    file.println(_log_data);                          // write data into the file,
    file.close();                                  // close the file,
    Serial.println(_log_data);
  }
  else {                                              // if the file isn't open, pop up an error message,
    Serial.println("error opening file");
  }
}
