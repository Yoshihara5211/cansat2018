#include "compass.h"
Compass::Compass() {
}

Compass::~Compass() {
}

void Compass::setupCompass(unsigned char REG_ADR, unsigned char DATA) {
  Wire.begin();
  Wire.beginTransmission(HMC5883L_ADR);
  Wire.write(REG_ADR);
  Wire.write(DATA);
  Wire.endTransmission(false);
}

void Compass::calibration() {
  x = y = z = 0;
  x_max = -20000;
  x_min = 20000;
  y_max = -20000;
  y_min = 20000;
  z_max = -20000;
  z_min = 20000;
  for (int i = 0; i < 8000; i++) {
    Wire.end();
    Wire.begin();
    // delay(250);                            //WAIT DATA SET TIME
    X_MSB = I2C_READ(0x03);
    X_LSB = I2C_READ(0x04);
    Z_MSB = I2C_READ(0x05);
    Z_LSB = I2C_READ(0x06);
    Y_MSB = I2C_READ(0x07);
    Y_LSB = I2C_READ(0x08);

    X_12 = X_MSB;
    X_12 = ((X_12 << 8) & 0xFF00) | X_LSB; //SHIFT & GET 12bit DATA WITH MSB
    Z_12 = Z_MSB;
    Z_12 = ((Z_12 << 8) & 0xFF00) | Z_LSB; //SHIFT & GET 12bit DATA WITH MSB
    Y_12 = Y_MSB;
    Y_12 = ((Y_12 << 8) & 0xFF00) | Y_LSB; //SHIFT & GET 12bit DATA WITH MSB
    double x_tmp = X_12;
    double y_tmp = Y_12;
    double z_tmp = Z_12;

    if (x_tmp > x_max) {
      x_max = x_tmp;
    }
    else if (x_tmp < x_min) {
      x_min = x_tmp;
    }
    if (y_tmp > y_max) {
      y_max = y_tmp;
    }
    else if (y_tmp < y_min) {
      y_min = y_tmp;
    }

    if (z_tmp > z_max) {
      z_max = z_tmp;
    }
    else if (z_tmp < z_min) {
      z_min = z_tmp;
    }
  }
  x_cal = ((double)x_max + (double)x_min) / 2;
  y_cal = ((double)y_max + (double)y_min) / 2;
  z_cal = ((double)z_max + (double)z_min) / 2;

  //  Serial.print(x_cal);
  // Serial.print(y_cal);
  // Serial.println(z_cal);
}

void Compass::calibration2() {
  x = y = z = 0;
  x_max = -20000;
  x_min = 20000;
  y_max = -20000;
  y_min = 20000;
  z_max = -20000;
  z_min = 20000;
  for (int i = 0; i < 8000; i++) {
    Wire.end();
    Wire.begin();
    // delay(250);                            //WAIT DATA SET TIME
    X_MSB = I2C_READ(0x03);
    X_LSB = I2C_READ(0x04);
    Z_MSB = I2C_READ(0x05);
    Z_LSB = I2C_READ(0x06);
    Y_MSB = I2C_READ(0x07);
    Y_LSB = I2C_READ(0x08);

    X_12 = X_MSB;
    X_12 = ((X_12 << 8) & 0xFF00) | X_LSB; //SHIFT & GET 12bit DATA WITH MSB
    Z_12 = Z_MSB;
    Z_12 = ((Z_12 << 8) & 0xFF00) | Z_LSB; //SHIFT & GET 12bit DATA WITH MSB
    Y_12 = Y_MSB;
    Y_12 = ((Y_12 << 8) & 0xFF00) | Y_LSB; //SHIFT & GET 12bit DATA WITH MSB
    double x_tmp = X_12;
    double y_tmp = Y_12;
    double z_tmp = Z_12;

    if (x_tmp > x_max) {
      x_max = x_tmp;
    }
    else if (x_tmp < x_min) {
      x_min = x_tmp;
    }
    if (y_tmp > y_max) {
      y_max = y_tmp;
    }
    else if (y_tmp < y_min) {
      y_min = y_tmp;
    }

    if (z_tmp > z_max) {
      z_max = z_tmp;
    }
    else if (z_tmp < z_min) {
      z_min = z_tmp;
    }
  }
  x_cal = ((double)x_max + (double)x_min) / 2;
  y_cal = ((double)y_max + (double)y_min) / 2;
  z_cal = ((double)z_max + (double)z_min) / 2;

  countCali++;
  if (countCali < 10) {
    rightMotor.go(255);
    leftMotor.go(255);
  }
  else if (countCali >= 10 && countCali < 14) {
    rightMotor.brake();
    leftMotor.brake();
  }
  else if (countCali >= 14 && countCali < 24) {
    rightMotor.back(255);
    leftMotor.back(255);
  }
  else if (countCali >= 25) {
    rightMotor.go(255);
    leftMotor.go(255);
  }
  else {
    rightMotor.stopSlowly2();
    leftMotor.stopSlowly2();
  }
}

unsigned char Compass::I2C_READ(unsigned char REG_ADR) {
  Wire.beginTransmission(HMC5883L_ADR);  // 指定したアドレス（HMC5883L_ADR 0x1E）のI2Cスレーブに対して送信処理開始
  Wire.write(REG_ADR);  // マスタがスレーブに送信するデータをキューに入れるときに使用,beginTransmission()とendTransmission()の間で実行
  Wire.endTransmission(false);  // スレーブデバイスに対する送信を完了，falseに設定するとrestartメッセージをリクエストのあと送信しコネクションを維持
  Wire.requestFrom(HMC5883L_ADR, 1);  // デバイス(アドレスHMC5883L_ADR)に対し1バイトを要求
  return  Wire.read();  // マスタデバイスでは、requestFrom()を実行したあと、スレーブから送られてきたデータを読み取るときに使用
}

void Compass::I2C_WRITE(unsigned char REG_ADR, unsigned char DATA)
{
  Wire.beginTransmission(HMC5883L_ADR);
  Wire.write(REG_ADR);
  Wire.write(DATA);
  Wire.endTransmission(false);
}

void Compass::readCompass(double ax, double ay, double az) {
  Wire.end();
  Wire.begin();
  // delay(250);                            //WAIT DATA SET TIME
  X_MSB = I2C_READ(0x03);
  X_LSB = I2C_READ(0x04);
  Z_MSB = I2C_READ(0x05);
  Z_LSB = I2C_READ(0x06);
  Y_MSB = I2C_READ(0x07);
  Y_LSB = I2C_READ(0x08);

  X_12 = X_MSB;
  X_12 = ((X_12 << 8) & 0xFF00) | X_LSB; //SHIFT & GET 12bit DATA WITH MSB
  Z_12 = Z_MSB;
  Z_12 = ((Z_12 << 8) & 0xFF00) | Z_LSB; //SHIFT & GET 12bit DATA WITH MSB
  Y_12 = Y_MSB;
  Y_12 = ((Y_12 << 8) & 0xFF00) | Y_LSB; //SHIFT & GET 12bit DATA WITH MSB


  /////////////////////////////////////////////////////
  // roll軸，pitch軸の回転を考慮できていない，加速度センサからのx,y,zと磁束密度x,y,zを使う必要あり
  //
  X_DOUBLE = X_12 - x_cal;                     //CONVERT TO DOUBLE (FOR atan2)
  Y_DOUBLE = Y_12 - y_cal;                     //CONVERT TO DOUBLE (FOR atan2)
  Z_DOUBLE = Z_12 - z_cal;
  //


  roll = atan2(ay, az);
  pitch = atan2(-ax, ay * sin(roll) + az * cos(roll));

  //RAD_RESULT = atan2(Y_DOUBLE, X_DOUBLE); //GET RADIAN
  RAD_RESULT = atan2(Z_DOUBLE * sin(roll) - Y_DOUBLE * cos(roll), X_DOUBLE * cos(pitch) + Y_DOUBLE * sin(pitch) * sin(roll) + Z_DOUBLE * sin(pitch) * cos(roll)); //GET RADIAN
  deg = RAD_RESULT * 180 / M_PI; //GET DEGREE
  /////////////////////////////////////////////////////



}
