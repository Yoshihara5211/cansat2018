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
//#include "mike.h"

class Cansat {
  public:
    // オブジェクト生成

    // ピン指定
    Motor leftMotor = Motor(LEFT_MOTOR_FIN_PIN, LEFT_MOTOR_RIN_PIN);
    Motor rightMotor = Motor(RIGHT_MOTOR_FIN_PIN, RIGHT_MOTOR_RIN_PIN);
    Light light = Light(LIGHT_PIN);
    Acc acc = Acc(ACC_X_PIN, ACC_Y_PIN, ACC_Z_PIN);
    Mic mic1 = Mic(MIC_PIN1);
    Mic mic2 = Mic(MIC_PIN2);
    Mic mic3 = Mic(MIC_PIN3);
    Mic mic4 = Mic(MIC_PIN4);

    // シリアル通信
    Radio radio;
    Gps gps;

    // SPI通信
    Sd sd;

    // I2C通信
    Compass compass;

    // メイン関数
    void setup();  // 各センサ，コンポーネント，シーケンスのsetup(ここでキャリブレーションをしたい)
    void sequence();  // シーケンス制御(シーケンス関数を回す)
    void writeSd();  // ログ保存
    void sendXbee();  // 無線送信

    // setup関数
    void setGoal();  // ゴール座標設定関数

    // sequence関数
    void preparing();
    void flying();
    void dropping();
    void landing();
    void running();
    ///////////////////
    void stucking();
    void guidance1(float nowLat, float nowLon, float nowDeg, float goalLat, float goalLon);  // GPS + COMPASS
    void guidance2(float nowLat, float nowLon, float goalLat, float goalLon);  // GPS only　<<<<<---小菅
    void guidance3();  // <<<<<----富吉
    //////////////////////////////////////
    // 音の情報を取得する関数
    void sound_read();
    // 最大音量、最大音周波数、どの向きから音が来てるか(1→2→3→4、前→右→後→左)
    int maxvol,maxfreq,direc;
    // 地磁気センサ＋マイクのアルゴリズム
    void guidance3();
    // 地磁気センサなしでの走行アルゴリズム
    void guidance4();
    ////////////////////////////////////
    // 音源に近づくアルゴリズム
    // 各マイクセンサ(3つ or ４つ)から得られた各周波数帯における振幅値を引数に
    // 音源からの極座標検知関数
    // 機体の姿勢検知関数
    // 走行アルゴリズム関数
    /////////////////////////////////////
    //////////////////
    void goal();


    // 変数

};

#endif
