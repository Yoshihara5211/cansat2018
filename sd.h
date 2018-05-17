/**
  @file sd.h
  @author Hiroyuki Yoshihara
  @date Created: 20180501
*/
#ifndef _SD_H_
#define _SD_H_
#include <stdio.h>
#include <string.h>
#include "Arduino.h"
#include <SD.h>

class Sd {
  public:
  // 関数
  Sd();
  ~Sd();
  void setupSd();
  void printSd(String _dataIn);
  
  // 変数
  String dataSet;
  String dataIn;
  String allData;
  };

  #endif

