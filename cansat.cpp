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
  Serial.begin(9600);
  sd.setupSd();
  Serial.println("Sd is ok");
  radio.setupRadio();
  Serial.println("Radio is ok");
  gps.setupGps();
  Serial.println("Gps is ok");
  acc.setupAcc();
  Serial.println("Acc is ok");
  compass.setupCompass(0x02, 0x00);
  compass.calibration();
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
  compass.readCompass();
  Serial.println("Compass is ok");
  gps.readGps();
  Serial.println("Gps is ok");
  writeSd();
  Serial.println("log is ok");
  sendXbee();
  Serial.println("radio is ok");
}


//void Cansat::guidance1(float nowLat, float nowLon, float nowDeg, float goalLat, float goalLon) {
//  // Lon=経度=x
//  // Lat=緯度=y
//  // 地球の球体を考慮した座標値変換する必要あり
//  float deltaLon = (goalLon - nowLon) ;
//  float deltaLat = (goalLat - nowLat);
//  float distance = sqrt(pow(deltaLat, 2) + pow(deltaLon, 2));
//  // 機体座標に変換，回転行列使うよ，deg2radするよ
//  float bodyLon = deltaLon * cos(nowDeg / 180 * M_PI) + deltaLat * sin(nowDeg / 180 * M_PI); // [x'] =  [cos(th)     sin(th)] [x]
//  float bodyLat = deltaLon * sin(nowDeg / 180 * M_PI) + deltaLat * cos(nowDeg / 180 * M_PI); // [y']   [-sin(th)    cos(th)] [y]
//
//  // 機体座標系でのゴールまでの角度を計算
//  if (bodyLat > 0) {
//    float bodyAngle = fabs(atan(bodyLon / bodyLat)) * 180 / M_PI;
//  } else if (bodyLat < 0) {
//    float bodyAngle = 180 - fabs(atan(bodyLon / bodyLat)) * 180 / M_PI;
//  } else
//    float  bodyAngle = 90;
//}
//
//// ある角度以内なら真っ直ぐ，それ以外で右は右，左は左．
//if (bodyAngle < ANGLE_THRE) {
//  int direct = 0; //真っ直ぐ
//} else {
//  if (bodyLon >= 0) {
//    int direct = 1; //右
//  } else {
//    int direct = -1; //左
//  }
//}
//
//// モータの駆動
//if (_direct == 0) {
//  rightMotor.go(255);
//  leftMotor.go(255);
//} else if (_direct == 1) { //右
//  rightMotor.go(255 * (1 - _bodyAngle / 180));
//  leftMotor.go(255);
//} else if (_direct == -1) { //左
//  rightMotor.go(255);
//  leftMotor.go(170 * (1 - _bodyAngle / 180));
//}
//}
//
//
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
//  rightMotor.go(255);
//  leftMotor.go(255);
//  delay(10000);
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
//  mic1.FFT();
//  mic2.FFT();
//  mic3.FFT();
//  mic4.FFT();
//
//  int maxdb[4] = {0, 0, 0, 0}; // 各マイクが拾った音の大きさ
//  int freq[4] = {0, 0, 0, 0}; //各マイクが拾った音の高さ
//  int i, j; // for文用
//
//  // 2kHz未満の音絶対拾わないマン
//  for (i = 32; i < 64; i++) {
//
//    // なぜか音量がchar型でしかfftできないのでここでint型に変換
//    sprintf(mic1.buf, "%5d", mic1.data[i]);
//    sprintf(mic2.buf, "%5d", mic2.data[i]);
//    sprintf(mic3.buf, "%5d", mic3.data[i]);
//    sprintf(mic4.buf, "%5d", mic4.data[i]);
//    int i1 = atoi(mic1.buf);
//    int i2 = atoi(mic2.buf);
//    int i3 = atoi(mic3.buf);
//    int i4 = atoi(mic4.buf);
//    //変換完了
//
//    //maxdb[0],i1,freq[0]はマイク1の担当
//    if (maxdb[0] < i1) {
//      maxdb[0] = i1;
//      freq[0] = i;
//    }
//    if (maxdb[1] < i2) {
//      maxdb[1] = i2;
//      freq[1] = i;
//    }
//    if (maxdb[2] < i3) {
//      maxdb[2] = i3;
//      freq[2] = i;
//    }
//    if (maxdb[3] < i4) {
//      maxdb[3] = i4;
//      freq[3] = i;
//    }
//  }
//  //4つのマイクが拾った音の内
//  maxvol = 0, maxfreq = 0;
//  for (j = 0; j < 4; j++) {
//    if (maxvol < maxdb[j]) {
//      maxvol = maxdb[j];
//      maxfreq = freq[j];
//      direc = j + 1;
//    }
//  }
//}
////////////////////////////////////////////////////////////////
//  // 並び替え(バブルソート)→音が大きい順にvol,freq,numberを並び替える
//void Cansat::sound_sort() {
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
//}
///////////////////////////////////////////////////////////////////////////////////////////
//// 地磁気センサ＋マイクのアルゴリズム
//void Cansat::guidance3() {
//  vol[4] = {micf.maxvol, micr.maxvol, micb.maxvol, micl.maxvol}; // 各マイクが拾った音の大きさ
//  freq[4] = {micf.maxfreq, micr.maxfreq, micb.maxfreq, micl.maxfreq}; //各マイクが拾った音の高さ
//  number[4] = {1, 2, 3, 4};//各マイクの番号(1,2,3,4→前、右、後、左のつもり)
//
//  sound_sort();
//
//   if (vol[0] < 4) {
//    vol[0] = 0;
//    freq[0] = 0;
//    direc = 0;
//  }
// ここからは音の高さで方向検知するやつ
// 2290Hzから70Hz刻みでやってます。
//if(freq[0]==33){
//  Serial.println("12時");
//  }
//else if(freq[0]==34){
//  Serial.println("1時");
//}
//else if(freq[0]==35){
//  Serial.println("2時");
//}
//else if(freq[0]==36){
//  Serial.println("3時");
//}
//else if(freq[0]==37){
//  Serial.println("4時");
//}
//else if(freq[0]==38){
//  Serial.println("5時");
//}
//else if(freq[0]==39){
//  Serial.println("6時");
//}
//else if(freq[0]==40){
//  Serial.println("7時");
//}
//else if(freq[0]==41){
//  Serial.println("8時");
//}
//else if(freq[0]==42){
//  Serial.println("9時");
//}
//else if(freq[0]==43){
//  Serial.println("10時");
//}
//else if(freq[0]==44){
//  Serial.println("11時");
//}
//
//Serial.print("音のでかさは：");
//Serial.println(vol[0]);
//Serial.print("音の高さは：");
//if(freq[0]-33<0){
//Serial.print(0);
//}else{
//  Serial.print((freq[0]-33)*70+2290);
//}
//Serial.println(" Hz");
//}
//
///////////////////////////////////////////////////////////////////////////////////////////
//// 地磁気センサなしでの走行アルゴリズム(これでひとまず完成)
//void Cansat::guidance4() {
//  vol[4] = {micf.maxvol, micr.maxvol, micb.maxvol, micl.maxvol}; // 各マイクが拾った音の大きさ
//  freq[4] = {micf.maxfreq, micr.maxfreq, micb.maxfreq, micl.maxfreq}; //各マイクが拾った音の高さ
//  number[4] = {1, 2, 3, 4};//各マイクの番号(1,2,3,4→前、右、後、左のつもり)
//
//  sound_sort();
//
//  //　向き判定
//  if (vol[0] < 4) {
//    vol[0] = 0;
//    freq[0] = 0;
//    direc = 0;
//  }
//  else if (vol[0] > 5) {
//    if ((float)vol[0] / (float)vol[1] > 2.5) {
//      direc = 2 * number[0] - 1;
//    }
//    else if ((number[0] == 1 && number[1] == 2) || (number[0] == 2 && number[1] == 1)) {
//      direc = 2;
//    }
//    else if ((number[0] == 2 && number[1] == 3) || (number[0] == 3 && number[1] == 2)) {
//      direc = 4;
//    }
//    else if ((number[0] == 3 && number[1] == 4) || (number[0] == 4 && number[1] == 3)) {
//      direc = 6;
//    }
//    else if ((number[0] == 4 && number[1] == 1) || (number[0] == 1 && number[1] == 4)) {
//      direc = 8;
//    } else {
//      direc = 9; // なんかバグってます
//    }
//  } else {
//    direc = 2 * number[0] - 1;
//  }
//
//  //モーター系
//  if (direc == 0) {
//    Serial.println("No sound detected");
//    rightMotor.stop();
//    leftMotor.stop();
//  }
//  else if (direc == 1) {
//    Serial.println("front");
//    rightMotor.go(255);
//    leftMotor.go(255);
//  }
//  else if (direc == 2) {
//    Serial.println("front/right");
//    rightMotor.go(122.5);
//    leftMotor.go(255);
//  }
//  else if (direc == 3) {
//    Serial.println("right");
//    rightMotor.stop();
//    leftMotor.go(255);
//  }
//  else if (direc == 4) {
//    Serial.println("right/back");
//    rightMotor.back(122.5);
//    leftMotor.back(255);
//  }
//  else if (direc == 5) {
//    Serial.println("back");
//    rightMotor.back(255);
//    leftMotor.back(255);
//  }
//  else if (direc == 6) {
//    Serial.println("left/back");
//    rightMotor.back(255);
//    leftMotor.back(122.5);
//  }
//  else if (direc == 7) {
//    Serial.println("left");
//    rightMotor.go(255);
//    leftMotor.stop();
//  }
//  else if (direc == 8) {
//    Serial.println("front/left");
//    rightMotor.go(255);
//    leftMotor.go(122.5);
//  }
//  else {
//    Serial.println("なんかバグってますよ");
//  }
//
//
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







