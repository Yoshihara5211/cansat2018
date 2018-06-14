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
  float Lon1 = nowLon;
  float Lat1 = nowLat;
  ////モータの駆動
  rightMotor.go(255);
  leftMotor.go(255);
  delay(10000);
  ////動いた後のGPSの値
  float Lon2 = gps.lon
               float Lat2 = gps.lat;
  float xvel2g = (goalLon - Lon2) / sqrt(pow(goalLon - Lon2, 2) + pow(goalLat - Lat2, 2));
  float yvel2g = (goalLat - Lat2) / sqrt(pow(goalLon - Lon2, 2) + pow(goalLat - Lat2, 2));
  float deg2g = atan(yvel2g / xvel2g);
  float xvel12 = (Lon2 - Lon1)sqrt(pow(goalLon - Lon2, 2) + pow(goalLat - Lat2, 2));
  float yvel12 = (Lat2 - Lat1)sqrt(pow(goalLon - Lon2, 2) + pow(goalLat - Lat2, 2));
  float deg12 = atan(yvel12 / xvel12);

  ////モータの駆動
  if (deg12 > deg2g) { //左を上げる
    rightMotor.go(255 * 0.8);
    leftMotor.go(255);
  } else if (deg2g > deg12) { //右を上げる
    rightMotor.go(255);
    leftMotor.go(255 * 0.8);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////


void Cansat::sound_read() {
  // FFT→結果はmic.data[]に格納
  mic1.FFT();
  mic2.FFT();
  mic3.FFT();
  mic4.FFT();
  mic1.soundRead();
  mic2.soundRead();
  mic3.soundRead();
  mic4.soundRead();
  

  int vol[4] = {mic1.maxvol, mic2.maxvol, mic3.maxvol, mic4.maxvol}; // 各マイクが拾った音の大きさ
  int freq[4] = {mic1.maxfreq, mic2.maxfreq, mic3.maxfreq, mic4.maxfreq}; //各マイクが拾った音の高さ
  int j; // for文用
  
  //maxvol→大きさ、maxfreq→高さ、direc→どっちから音が来てるか
  maxvol = 0, maxfreq = 0, direc = 0;
  for (j = 0; j < 4; j++) {
    if (maxvol < vol[j]) {
      maxvol = vol[j];
      maxfreq = freq[j];
      direc = j + 1;
    }
  }
  //音の大きさがある値より小さいときは0とする
  if (maxvol < 3) {
    maxvol = 0;
    maxfreq = 0;
    direc = 0;
  }
}


/////////////////////////////////////////////////////////////////////////////////////////
// 地磁気センサ＋マイクのアルゴリズム

void Cansat::guidance3() {

  sound_read();//ここで音の高さと音の大きさがわかる

  if (maxvol > SOUND_VOLUME) {

    // 東西南北8方向を検知させ、地磁気センサの値と合わせて音源へと向かわせる

    if (maxfreq == 36) {        //2500Hz
      //北へ
    } else if (maxfreq == 37) { //2570Hz
      //北東へ
    } else if (maxfreq == 38) { //2640Hz
      //東へ
    } else if (maxfreq == 39) { //2710Hz
      //南東へ
    } else if (maxfreq == 40) { //2780Hz
      //南へ
    } else if (maxfreq == 41) { //2850Hz
      //南西へ
    } else if (maxfreq == 42) { //2920Hz
      //西へ
    } else if (maxfreq == 43) { //2990Hz
      //北西へ
    } else {
      Serial.println("想定していない音を拾っています")
    }
  } else {//音が拾えてなかったら
    rightMotor.stopSlowly();
    leftMotor.stopSlowly();
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
// 地磁気センサなしでの走行アルゴリズム

void Cansat::guidance4() {

  sound_read(); // ここで音がどっちから飛んできたかが変数direcに入る

  if (maxvol > SOUND_VOLUME) {//音が拾えていたら

    // モーターの挙動
    if (direc == 1) { //front
      rightMotor.go(255);
      leftMotor.go(255);
    } else if (direc == 2) { //right
      leftMotor.go(255);
    } else if (direc == 3) { //left
      rightMotor.go(255);
    } else if (direc == 4) { //back
      rightMotor.back(255);
      leftMotor.go(255);
    } else {
      Serial.println("なんかバグってます(guidance4)")//通常動作でdirecは1,2,3,4のどれかになる予定
    }
  } else { //音が拾えてなかったら
    rightMotor.stopSlowly();
    leftMotor.stopSlowly();
  }
}









