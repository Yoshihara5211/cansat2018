

/**
  @file cansat2018.ino
  @brief CanSat Project 2018
  @author Hiroyuki Yoshihara
  @date Created: 20180413
*/

#include <stdio.h>
#include <string.h>
#include "Arduino.h"
#include "constant.h" //全ての定数はここ
#include "cansat.h"


Cansat cansat;


void setup() {
  cansat.setup();
}

void loop() {
  cansat.sensor();
  delay(50); // 1機目
//  delay(70); // 2機目
  cansat.sequence();
  delay(50); // 1機目
//  delay(70); // 2機目
}
