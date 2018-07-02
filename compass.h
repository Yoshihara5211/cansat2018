#ifndef _COMPASS_H_
#define _COMPASS_H_
#include <stdio.h>
#include <string.h>
#include "Arduino.h"
#include <Wire.h>
#include <math.h>
#define HMC5883L_ADR 0x1E //7bit ADDRESS


class Compass {
  public:
    // 関数
    Compass();
    ~Compass();
    void setupCompass(unsigned char REG_ADR, unsigned char DATA);
    void calibration();
    unsigned char I2C_READ(unsigned char REG_ADR);
    void I2C_WRITE(unsigned char REG_ADR, unsigned char DATA);
    void readCompass();
    
    // 変数
    unsigned char X_MSB;
    unsigned char X_LSB;
    unsigned char Y_MSB;
    unsigned char Y_LSB;
    unsigned char Z_MSB;
    unsigned char Z_LSB;
    int X_12;
    int Z_12;
    int Y_12;
    double X_DOUBLE;
    double Y_DOUBLE;
    double Z_DOUBLE;
    double RAD_RESULT;
    double deg;
    int x;
    int y;
    int z;
    int x_max;
    int y_max;
    int x_min;
    int y_min;
    int z_max;
    int z_min;
    double x_cal;
    double y_cal;
    double z_cal;
};
#endif
