/**
  @file cansat.cpp
  @author Hiroyuki Yoshihara
  @date Created: 20170518
*/
#include "cansat.h"


void Cansat::guidance1(float nowLat, float nowLon, float nowDeg, float goalLat, float goalLon) {
  // Lon=経度=x
  // Lat=緯度=y
  // メートル変換するよ，地球の球体を考慮した座標値変換する必要あり
  float deltaLon = (goalLon - nowLon) * 100000;
  float deltaLat = (goalLat - nowLat) * 100000; 
  float distance = sqrt(pow(deltaLat, 2) + pow(deltaLon, 2));
  // 機体座標に変換，回転行列使うよ，deg2radするよ
  float bodyLon = deltaLon * cos(nowDeg / 180 * M_PI) + deltaLat * sin(nowDeg / 180 * M_PI); // [x'] =  [cos(th)     sin(th)] [x]
  float bodyLat = deltaLon * sin(nowDeg / 180 * M_PI) + deltaLat * cos(nowDeg / 180 * M_PI); // [y']   [-sin(th)    cos(th)] [y]

  // 機体座標系でのゴールまでの角度を計算
  if (bodyLat > 0) {
    float bodyAngle = fabs(atan(bodyLon / bodyLat)) * 180 / M_PI;
  } else if (bodyLat < 0) {
     float bodyAngle = 180 - fabs(atan(bodyLon / bodyLat)) * 180 / M_PI;
  } else 
   float  bodyAngle = 90;
  }
  
  // ある角度以内なら真っ直ぐ，それ以外で右は右，左は左．
  if (bodyAngle < ANGLE_THRE) {
     int direct = 0; //真っ直ぐ
  } else {
    if (bodyLon >= 0) {
      int direct = 1; //右
    } else {
      int direct = -1; //左
    }
  }
  
  // モータの駆動
  if (_direct == 0) {
    rightMotor.go(255);
    leftMotor.go(255);
  } else if (_direct == 1) { //右
    rightMotor.go(255 * (1 - _bodyAngle / 180));
    leftMotor.go(255);
  } else if (_direct == -1) { //左
    rightMotor.go(255);
    leftMotor.go(170 * (1 - _bodyAngle / 180));
  }
}


///////////////////////////////////////////////////////////////////////////////////////
/**
  @void guidance2
  @author Kosuge
  @date Created: 20170529
*/
void Cansat::guidance2(float nowLat, float nowLon, float goalLat, float goalLon) {
  // Lon=経度=x
  // Lat=緯度=y
  ////自分のGPSの値　初期値
  float Lon1=nowLon;
  float Lat1=nowLat;
  ////モータの駆動
  rightMotor.go(255);
  leftMotor.go(255);
  delay(10000);
  ////動いた後のGPSの値
  float Lon2=gps.lon
  float Lat2=gps.lat;
  float xvel2g=(goalLon-Lon2)/sqrt(pow(goalLon-Lon2, 2) + pow(goalLat-Lat2, 2));
  float yvel2g=(goalLat-Lat2)/sqrt(pow(goalLon-Lon2, 2) + pow(goalLat-Lat2, 2));
  float deg2g=atan(yvel2g/xvel2g);
  float xvel12=(Lon2-Lon1)sqrt(pow(goalLon-Lon2, 2) + pow(goalLat-Lat2, 2));
  float yvel12=(Lat2-Lat1)sqrt(pow(goalLon-Lon2, 2) + pow(goalLat-Lat2, 2)); 
  float deg12=atan(yvel12/xvel12);

////モータの駆動
  if (deg12>deg2g) {//左を上げる
    rightMotor.go(255*0.8);
    leftMotor.go(255);
  } else if (deg2g>deg12) { //右を上げる
    rightMotor.go(255);
    leftMotor.go(255*0.8);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////














  
