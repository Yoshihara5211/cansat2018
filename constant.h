/**
* @file cansat.h
* @author Hiroyuki Yoshihara
* @date Created: 20180415
*/
#ifndef _CONSTANT_H_
#define _CONSTANT_H_

//ピン番号指定  
#define LEFT_MOTOR_VREF_PIN 4
#define LEFT_MOTOR_IN1_PIN 3
#define LEFT_MOTOR_IN2_PIN 2
#define RIGHT_MOTOR_VREF_PIN 45
#define RIGHT_MOTOR_IN1_PIN 44
#define RIGHT_MOTOR_IN2_PIN 46
#define RED_LED_PIN A9
#define BLUE_LED_PIN A5
#define GREEN_LED_PIN A4
#define BUZZER_PIN A15
#define ACC_X_PIN A8
#define ACC_Y_PIN A7
#define ACC_Z_PIN A6
#define LIGHT_PIN A3
#define MIC_FRONT_PIN A0
#define MIC_RIGHT_PIN A14
#define MIC_LEFT_PIN A1 
#define MIC_BACK_PIN A2

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
#define ANGLE_THRE 20



#endif

