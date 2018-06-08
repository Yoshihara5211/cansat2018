/**
* @file cansat.h
* @author Hiroyuki Yoshihara
* @date Created: 20180415
*/
#ifndef _CONSTANT_H_
#define _CONSTANT_H_

//ピン番号指定
#define LIGHT_PIN 1  
#define LEFT_MOTOR_FIN_PIN 2
#define LEFT_MOTOR_RIN_PIN 3
#define RIGHT_MOTOR_FIN_PIN 4
#define RIGHT_MOTOR_RIN_PIN 5
#define MIC_FRONT_PIN 6
#define MIC_RIGHT_PIN 7
#define MIC_LEFT_PIN 8 
#define MIC_BACK_PIN 9

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
#define SOUND_VOLUME 4



#endif

