/**
* @file cansat.h
* @author Hiroyuki Yoshihara
* @date Created: 20180415
*/
#ifndef _CONSTANT_H_
#define _CONSTANT_H_

//ピン番号指定  
#define LEFT_MOTOR_FIN_PIN 1
#define LEFT_MOTOR_RIN_PIN 2
#define RIGHT_MOTOR_FIN_PIN 3
#define RIGHT_MOTOR_RIN_PIN 4
#define RED_LED_PIN 10
#define YELLOW_LED_PIN 11
#define GREEN_LED_PIN 12
#define BUZZER_PIN 13
#define ACC_X_PIN A0
#define ACC_Y_PIN A1
#define ACC_Z_PIN A2
#define LIGHT_PIN A3
#define MIC_FRONT_PIN A8
#define MIC_RIGHT_PIN A9
#define MIC_LEFT_PIN A10 
#define MIC_BACK_PIN A11

#define BEAT 100   // 音の長さを指定


//シークエンス
#define PREPARING 0   // 電源オン
#define FLYING 1  // 格納検知
#define DROPING 2  // 放出検知
#define LANDING 3  // 着地検知
#define RUNNING 4  // 走行アルゴリズム
#define GOAL 5  // ゴール検知
#define STUCKING 6  // スタック検知

//閾値
#define ANGLE_THRE 1



#endif

