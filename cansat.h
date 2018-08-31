/**
  @file cansat.h
  @author Hiroyuki Yoshihara
  @date Created: 20170518
*/
#ifndef _CANSAT_H_
#define _CANSAT_H_
#include <stdio.h>
#include <string.h>
#include "Arduino.h"
#include "constant.h" //定数まとめ(ピン番号等)
#include "motor.h"
#include "light.h"
#include "gps.h"
#include "sd.h"
#include "radio.h"
#include "acc.h"
#include "compass.h"
#include "mic_module_fft.h"

class Cansat {
  public:
    Cansat();
    ~Cansat();
    // オブジェクト生成
    // ピン指定
    Motor leftMotor = Motor(LEFT_MOTOR_VREF_PIN, LEFT_MOTOR_IN1_PIN, LEFT_MOTOR_IN2_PIN);
    Motor rightMotor = Motor(RIGHT_MOTOR_VREF_PIN, RIGHT_MOTOR_IN1_PIN, RIGHT_MOTOR_IN2_PIN);
    Light light = Light(LIGHT_PIN);
    Acc acc = Acc(ACC_X_PIN, ACC_Y_PIN, ACC_Z_PIN);
    MIC micf = MIC(MIC_FRONT_PIN);
    MIC micr = MIC(MIC_RIGHT_PIN);
    MIC micl = MIC(MIC_LEFT_PIN);
    MIC micb = MIC(MIC_BACK_PIN);

    // シリアル通信
    Radio radio;
    Gps gps;

    // SPI通信
    Sd sd;

    // I2C通信
    Compass compass;

    // cansat2018関数
    void setup();  // 各センサ，コンポーネント，シーケンスのsetup(ここでキャリブレーションをしたい)
    void sensor();
    void sequence();  // シーケンス制御(シーケンス関数を回す




    // setup()構成関数
    void setGoal(float lon, float lat);  // ゴール座標設定関数

    // sensor()構成関数
    void writeSd();  // ログ保存
    void sendXbee();  // 無線送信

    // sequence()構成関数
    void test();
    void preparing();
    void flying();
    void dropping();
    void landing();
    void running();
    ////////////// running()構成関数
    void guidance1(float nowLon, float nowLat, float nowDeg, float goalLon, float goalLat);
    //    void guidance2(float nowLat, float nowLon, float goalLat, float goalLon);
    //    void guidance3();
    //    void guidance4();
    //////////////
    void goal();

    // 変数
    int laststate = 0;
    int state = 0;

    float destLon;
    float destLat;

    int countPreLoop = 0;
    int countFlyLoop = 0;
    int countDropLoop = 0;

    int preparingTime = 0;
    int flyingTime = 0;
    int droppingTime = 0;
    int landingTime = 0;
    int runningTime = 0;
    int stuckingTime = 0;

    float deltaLon = 0;
    float deltaLat = 0;
    float distance = 0;
    float bodyLon = 0;
    float bodyLat = 0;
    int bodyAngle = 0;
    int direct = 0;
};

#endif
