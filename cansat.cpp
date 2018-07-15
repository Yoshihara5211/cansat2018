/**
  @file cansat.cpp
  @author Hiroyuki Yoshihara
  @date Created: 20170518
*/
#include "cansat.h"

Cansat::Cansat() {
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
}

Cansat::~Cansat() {
}

void Cansat::setup() {
  setGoal(139.657881, 35.554789);  // ゴール設定(矢上グラウンド奥)
  
  Serial.begin(9600);

  sd.setupSd();
  Serial.println("Sd is ok");

  radio.setupRadio();
  Serial.println("Radio is ok");

  gps.setupGps();
  Serial.println("Gps is ok");

  digitalWrite(RED_LED_PIN, HIGH);
  tone(BUZZER_PIN, 131, 1000);
  acc.setupAcc();
  digitalWrite(RED_LED_PIN, LOW);
  Serial.println("Acc is ok");

  digitalWrite(YELLOW_LED_PIN, HIGH);
  tone(BUZZER_PIN, 523, 1000);
  compass.setupCompass(0x02, 0x00);
  compass.calibration();
  digitalWrite(YELLOW_LED_PIN, LOW);
  Serial.println("Compass is ok");
}

void Cansat::test() {
  Serial.println("---------------------------------------------------------------");
  micf.FFT();
  micr.FFT();
  micl.FFT();
  micb.FFT();
  micf.soundRead();
  micr.soundRead();
  micl.soundRead();
  micb.soundRead();
  Serial.println("Mic is ok");

  light.readLight();
  Serial.println("Light is ok");

  acc.readAcc();
  Serial.println("Acc is ok");

  compass.readCompass(acc.ax, acc.ay, acc.az);
  Serial.println("Compass is ok");

  gps.readGps();
  Serial.println("Gps is ok");

  writeSd();
  Serial.println("log is ok");

  sendXbee();
  Serial.println("radio is ok");

  guidance1(gps.lon, gps.lat, compass.deg, destLon, destLat);
}

void Cansat::setGoal(float lon, float lat){
  destLon = lon * 100000;
  destLat = lat * 100000;
  }

void Cansat::preparing() { // State = 0
  if (preparingTime == 0) {
    preparingTime = millis();
  }
}

void Cansat::guidance1(float nowLon, float nowLat, float nowDeg, float goalLon, float goalLat) {
  // Lon=経度=x
  // Lat=緯度=y
  // 地球の球体を考慮した座標値変換する必要あり
   deltaLon = (goalLon - nowLon) ;
   deltaLat = (goalLat - nowLat);
   distance = sqrt(pow(deltaLat, 2) + pow(deltaLon, 2));
  // 機体座標に変換，回転行列使うよ，deg2radするよ
   bodyLon = deltaLon * cos(nowDeg / 180 * M_PI) + deltaLat * sin(nowDeg / 180 * M_PI); // [x'] =  [cos(th)     sin(th)] [x]
   bodyLat = deltaLon * sin(nowDeg / 180 * M_PI) + deltaLat * cos(nowDeg / 180 * M_PI); // [y']   [-sin(th)    cos(th)] [y]

  // 機体座標系でのゴールまでの角度を計算
  if (bodyLat > 0) {
    bodyAngle = fabs(atan(bodyLon / bodyLat)) * 180 / M_PI;
  } else if (bodyLat < 0) {
     bodyAngle = 180 - fabs(atan(bodyLon / bodyLat)) * 180 / M_PI;
  } else {
      bodyAngle = 90;
  }

  // ある角度以内なら真っ直ぐ，それ以外で右は右，左は左．
  if (bodyAngle < ANGLE_THRE) {
     direct = 0; //真っ直ぐ
  } else {
    if (bodyLon >= 0) {
       direct = 1; //右
    } else {
       direct = -1; //左
    }
  }

  // モータの駆動
  if (direct == 0) {
    rightMotor.go(255);
    leftMotor.go(255);
  } else if (direct == 1) { //右
    rightMotor.go(255 * (1 - bodyAngle / 180));
    leftMotor.go(255);
  } else if (direct == -1) { //左
    rightMotor.go(255);
    leftMotor.go(255 * (1 - bodyAngle / 180));
  }
}



/////////////////////////////////////////////////////////////////////////////////////////
///**
//  @void guidance2
//  @author Kosuge
//  @date Created: 20170529
//*/
//void Cansat::guidance2(float nowLat, float nowLon, float goalLat, float goalLon) {
//  // Lon=経度=x
//  // Lat=緯度=y
//  ////自分のGPSの値　初期値
//  float Lon1 = nowLon;
//  float Lat1 = nowLat;
//  ////モータの駆動
//t1 =millis();
//  if(millis()-t1<10000){
//  rightMotor.go(255);
//  leftMotor.go(255);
//}else{
//break;
//}
//  ////動いた後のGPSの値
//  float Lon2 = gps.lon
//               float Lat2 = gps.lat;
//  float xvel2g = (goalLon - Lon2) / sqrt(pow(goalLon - Lon2, 2) + pow(goalLat - Lat2, 2));
//  float yvel2g = (goalLat - Lat2) / sqrt(pow(goalLon - Lon2, 2) + pow(goalLat - Lat2, 2));
//  float deg2g = atan(yvel2g / xvel2g);
//  float xvel12 = (Lon2 - Lon1)sqrt(pow(goalLon - Lon2, 2) + pow(goalLat - Lat2, 2));
//  float yvel12 = (Lat2 - Lat1)sqrt(pow(goalLon - Lon2, 2) + pow(goalLat - Lat2, 2));
//  float deg12 = atan(yvel12 / xvel12);
//
//  ////モータの駆動
//  if (deg12 > deg2g) { //左を上げる
//    rightMotor.go(255 * 0.8);
//    leftMotor.go(255);
//  } else if (deg2g > deg12) { //右を上げる
//    rightMotor.go(255);
//    leftMotor.go(255 * 0.8);
//  }
//}
///////////////////////////////////////////////////////////////////////////////////////////
//void Cansat::sound_read() {
//    // ここでどのマイクがどの高さの音をどの程度の大きさで拾っているのかを判定している
//  mic1.FFT();
//  mic2.FFT();
//  mic3.FFT();
//  mic4.FFT();
//  mic1.soundRead();
//  mic2.soundRead();
//  mic3.soundRead();
//  mic4.soundRead();
//
//  vol[4] = {mic1.maxvol, mic2.maxvol, mic3.maxvol, mic4.maxvol}; // 各マイクが拾った音の大きさ
//  freq[4] = {mic1.maxfreq, mic2.maxfreq, mic3.maxfreq, mic4.maxfreq}; //各マイクが拾った音の高さ
//  number[4] = {1, 2, 3, 4};//各マイクの番号(1,2,3,4→前、右、後、左のつもり)
//  // 並び替え(バブルソート)→音が大きい順にvol,freq,numberを並び替える
//  int i, j, temp;
//  for (i = 0; i < 3; i++) {
//    for (j = 3; j > i; j--) {
//      if (vol[j - 1] < vol[j]) {
//        temp = vol[j - 1];
//        vol[j - 1] = vol[j];
//        vol[j] = temp;
//        temp = freq[j - 1];
//        freq[j - 1] = freq[j];
//        freq[j] = temp;
//        temp = number[j - 1];
//        number[j - 1] = number[j];
//        number[j] = temp;
//      }
//    }
//  }
//
//  // 以下三行は場合によっては不要
//  maxvol=vol[0];
//  maxfreq=freq[0];
//  maxnumber=number[0]
//}
///////////////////////////////////////////////////////////////////////////////////////////
//// 地磁気センサ＋マイクのアルゴリズム
//void Cansat::guidance3() {
//
//  sound_read();
//
//  if (maxvol < 5) {
//    // sound_read()で音が取れてないときの例外処理
//  }
//  // この後cansatに東西南北8方向を検知させ、地磁気センサの値と合わせて音源へと向かわせる
//}
//
///////////////////////////////////////////////////////////////////////////////////////////
//// 地磁気センサなしでの走行アルゴリズム
//void Cansat::guidance4() {
//
//  sound_read();
//
//  if (maxvol < 5) {
//    // sound_read()で音が取れてないときの例外処理
//  }
//  // この後cansatに東西南北8方向を検知させ、地磁気センサの値と合わせて音源へと向かわせる
//}








// millis, state, light, lat, lon, ax, ay, az, deg, mic...
void Cansat::writeSd() {
  String log_data = String(millis()) + ", "
                    + String(state) + ", "
                    + String(light.lightValue) + ", "
                    + String(gps.lat) + ", "
                    + String(gps.lon) + ", "
                    + String(acc.ax) + ", "
                    + String(acc.ay) + ", "
                    + String(acc.az) + ", "
                    + String(compass.deg) + ", "
                    + String(micf.maxfreq) + ", "
                    + String(micf.maxvol) + ", "
                    + String(micr.maxfreq) + ", "
                    + String(micr.maxvol) + ", "
                    + String(micl.maxfreq) + ", "
                    + String(micl.maxvol) + ", "
                    + String(micb.maxfreq) + ", "
                    + String(micb.maxvol);
  sd.printSd(log_data);
}

void Cansat::sendXbee() {
  String send_data = String(millis()) + ","
                     + String(state) + ","
                     + String(light.lightValue) + ","
                     + String(gps.lat) + ","
                     + String(gps.lon) + ","
                     + String(acc.ax) + ","
                     + String(acc.ay) + ","
                     + String(acc.az) + ","
                     + String(compass.deg) + ","
                     + String(micf.maxfreq) + ","
                     + String(micf.maxvol) + ","
                     + String(micr.maxfreq) + ","
                     + String(micr.maxvol) + ","
                     + String(micl.maxfreq) + ","
                     + String(micl.maxvol) + ","
                     + String(micb.maxfreq) + ","
                     + String(micb.maxvol) + ","
                     + "e";
  radio.sendData(send_data);
}







