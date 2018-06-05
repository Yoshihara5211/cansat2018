#include "compass.h"
Compass::Compass() {
}

Compass::~Compass() {
}

void Compass::setupCompass(unsigned char REG_ADR, unsigned char DATA) {
  Wire.beginTransmission(HMC5883L_ADR);
  Wire.write(REG_ADR);
  Wire.write(DATA);
  Wire.endTransmission(false);
}

unsigned char Compass::I2C_READ(unsigned char REG_ADR) {
  Wire.beginTransmission(HMC5883L_ADR);  // 指定したアドレス（HMC5883L_ADR 0x1E）のI2Cスレーブに対して送信処理開始
  Wire.write(REG_ADR);  // マスタがスレーブに送信するデータをキューに入れるときに使用,beginTransmission()とendTransmission()の間で実行
  Wire.endTransmission(false);  // スレーブデバイスに対する送信を完了，falseに設定するとrestartメッセージをリクエストのあと送信しコネクションを維持
  Wire.requestFrom(HMC5883L_ADR, 1);  // デバイス(アドレスHMC5883L_ADR)に対し1バイトを要求
  return  Wire.read();  // マスタデバイスでは、requestFrom()を実行したあと、スレーブから送られてきたデータを読み取るときに使用
}

void Compass::readCompass() {
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
  X_DOUBLE = X_12;                       //CONVERT TO DOUBLE (FOR atan2)
  Y_DOUBLE = Y_12;                       //CONVERT TO DOUBLE (FOR atan2)
  //
  RAD_RESULT = atan2(Y_DOUBLE, X_DOUBLE); //GET RADIAN
  DEG_RESULT = RAD_RESULT * 180 / 3.142; //GET DEGREE
  /////////////////////////////////////////////////////

  
  
}

