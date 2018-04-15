/**
* @file cansat.h
* @author Hiroyuki Yoshihara
* @date Created: 20180415
*/
#ifndef _CONSTANT_H_
#define _CONSTANT_H_

//ピン番号指定
#define GPS_RX_PIN   // 適当なデジタルピン
#define GPS_TX_PIN   // 適当なデジタルピン
#define LIGHT_PIN   // 適当なアナログピン


//シークエンス
#define PREPARING 0   // 電源on
#define FLYING   // 格納検知
#define DROPING   // 放出検知
#define LANDING   // 着地検知
#define RUNNING   // 走行アルゴリズム
#define GOAL   // ゴール検知
#define STUCKING  // スタック検知

//閾値

#endif

