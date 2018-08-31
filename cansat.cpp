/**
  @file cansat.cpp
  @author Hiroyuki Yoshihara
  @date Created: 20170518
*/
#include "cansat.h"

Cansat::Cansat() {
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
}

Cansat::~Cansat() {
}

// setup関数
///////////////////////////////////////////////////////////////////////////////////
void Cansat::setup() {
  setGoal(139.657881, 35.554789);  // ゴール設定(矢上グラウンド奥)

  Serial.begin(9600);

  sd.setupSd();
  Serial.println("Sd is ok");

  radio.setupRadio();
  Serial.println("Radio is ok");

  gps.setupGps();
  Serial.println("Gps is ok");

  // 水平にしてキャリブレーション
  digitalWrite(RED_LED_PIN, HIGH);
//  tone(BUZZER_PIN, 131, 1000);
  acc.setupAcc();
  digitalWrite(RED_LED_PIN, LOW);
  Serial.println("Acc is ok");

  // roll,pitch,yawに回してキャリブレーション
  digitalWrite(BLUE_LED_PIN, HIGH);
//  tone(BUZZER_PIN, 523, 1000);
  compass.setupCompass(0x02, 0x00);
  compass.calibration();
  digitalWrite(BLUE_LED_PIN, LOW);
  Serial.println("Compass is ok");
}

void Cansat::setGoal(float lon, float lat) {
  destLon = lon * 100000;
  destLat = lat * 100000;
}
///////////////////////////////////////////////////////////////////////////////////

// sensor関数
///////////////////////////////////////////////////////////////////////////////////
void Cansat::sensor() {
  Serial.println("---------------------------------------------------------------");
  radio.getData();
  if (laststate != radio.radio_get_data - 48) {
  state = radio.radio_get_data - 48;
  laststate = radio.radio_get_data - 48;
  }
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

  if (state != FLYING) sendXbee();
  Serial.println("radio is ok");

  //  if (gps.lon < 1 && gps.lat < 1) {
  //    leftMotor.stop();
  //    rightMotor.stop();
  //  } else {
  //    guidance1(gps.lon, gps.lat, compass.deg, destLon, destLat);
  //  }
}

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
                    + String(micb.maxvol) + ","
                    + String(radio.radio_get_data);
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
                     + String(radio.radio_get_data) + ","
                     + "e";
  radio.sendData(send_data);
}
///////////////////////////////////////////////////////////////////////////////////

// sequence関数
///////////////////////////////////////////////////////////////////////////////////
void Cansat::sequence() {
  switch (state) {
    case PREPARING:
      preparing();
      break;
    case FLYING:
      flying();
      break;
    case DROPPING:
      dropping();
      break;
    case LANDING:
      landing();
      break;
    case RUNNING:
      running();
      break;
    case GOAL:
      goal();
      break;
  }
}

// State = 0
void Cansat::preparing() {
  if (preparingTime == 0) {
    preparingTime = millis();
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(BLUE_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
  }
  leftMotor.stop();
  rightMotor.stop();
  // 加速度から格納検知
  if (light.lightValue < LIGHT1_THRE) {
    countPreLoop++;
    if (countPreLoop > COUNT_LIGHT1_LOOP_THRE)  state = FLYING;
  }
  else {
    countPreLoop = 0;
  }
}

// State = 1
void Cansat::flying() {
  if (flyingTime == 0) {
    flyingTime = millis();
    tone(BUZZER_PIN, 523, 5000);
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(BLUE_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
  }
  rightMotor.stop();
  leftMotor.stop();
  // 光センサから放出検知
  if (light.lightValue > LIGHT2_THRE) {
    countFlyLoop++;
    if (countFlyLoop > COUNT_LIGHT2_LOOP_THRE) state = DROPPING;
  }
  else {
    countFlyLoop = 0;
  }
}

// State = 2
void Cansat::dropping() {
  if (droppingTime == 0) {
    droppingTime = millis();
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(BLUE_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, LOW);
  }
  // 落下フェーズでは第1パラシュート分離を行う
  if (landingTime != 0) {
    if (millis() - landingTime > RELEASING1_TIME_THRE) digitalWrite(RELEASING1_PIN, HIGH);
  }
  if (landingTime != 0) {
    if (millis() - landingTime > RELEASING1_TIME2_THRE) digitalWrite(RELEASING1_PIN, LOW);
  }
  // 加速度から着地検知
  if ((pow(acc.ax, 2) + pow(acc.ay, 2) + pow(acc.az, 2)) < (ACC_THRE, 2)) {
    countDropLoop++;
    if (countDropLoop > COUNT_ACC_LOOP_THRE) state = LANDING;
  }
  else {
    countDropLoop = 0;
  }
  // 時間から着地検知
  if (droppingTime != 0) {
    if (millis() - droppingTime > LANDING_TIME_THRE) state = LANDING;
  }
}

// State = 3
void Cansat::landing() {
  if (landingTime == 0) {
    landingTime = millis();
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(BLUE_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
  }
  // 着地フェーズでは第2パラシュート分離を行う
  digitalWrite(RELEASING2_PIN, HIGH);
  if (landingTime != 0) {
    if (millis() - landingTime > RELEASING2_TIME_THRE) digitalWrite(RELEASING2_PIN, LOW);
    state = RUNNING;
  }
}

// State = 4
void Cansat::running() {
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  // GPS無しでは停止
  if (gps.lat < 1 && gps.lon < 1) {
    leftMotor.stop();
    rightMotor.stop();
  }
  else {
    if (runningTime == 0) {
      runningTime = millis();
    }
    // 走行フェーズではガイダンス則に従う
    guidance1(gps.lon, gps.lat, compass.deg, destLon, destLat);
    if (fabs(destLon - gps.lon) <= GOAL_THRE && fabs(destLat - gps.lat) <= GOAL_THRE) state = GOAL;
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

// State = 5
void Cansat::goal() {
  leftMotor.stopSlowly();
  rightMotor.stopSlowly();
  digitalWrite(RED_LED_PIN, HIGH); delay(100);
  digitalWrite(BLUE_LED_PIN, HIGH); delay(100);
  digitalWrite(GREEN_LED_PIN, HIGH); delay(100);
  digitalWrite(RED_LED_PIN, LOW); delay(100);
  digitalWrite(BLUE_LED_PIN, LOW); delay(100);
  digitalWrite(GREEN_LED_PIN, LOW); delay(100);
}
///////////////////////////////////////////////////////////////////////////////////


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
  //  Serial.println("Mic is ok");
  //  light.readLight();
  //  Serial.println("Light is ok");
  //  acc.readAcc();
  //  Serial.println("Acc is ok");
  //  compass.readCompass();
  //  Serial.println("Compass is ok");
  //  gps.readGps();
  //  Serial.println("Gps is ok");
  //  writeSd();
  //  Serial.println("log_ok");
  //  sendXbee();
  //  Serial.println("radio.ok");

  // ここからは書き換えたもの//////////////////////////////////////////////////////////////////////
  int vol[4] = {micf.maxvol, micr.maxvol, micb.maxvol, micl.maxvol}; // 各マイクが拾った音の大きさ
  int freq[4] = {micf.maxfreq, micr.maxfreq, micb.maxfreq, micl.maxfreq}; //各マイクが拾った音の高さ
  int number[4] = {1, 2, 3, 4};//各マイクの番号(1,2,3,4→前、右、後、左のつもり)

  // 並び替え(バブルソート)→音が大きい順にvol,freq,numberを並び替える
  int i, j, temp;
  for (i = 0; i < 3; i++) {
    for (j = 3; j > i; j--) {
      if (vol[j - 1] < vol[j]) {
        temp = vol[j - 1];
        vol[j - 1] = vol[j];
        vol[j] = temp;
        temp = freq[j - 1];
        freq[j - 1] = freq[j];
        freq[j] = temp;
        temp = number[j - 1];
        number[j - 1] = number[j];
        number[j] = temp;
      }
    }
  }

  //　向き判定
  if (vol[0] < 4) {
    vol[0] = 0;
    freq[0] = 0;
    direct = 0;
  }
  else if (vol[0] > 5) {
    if ((float)vol[0] / (float)vol[1] > 1.5) {
      direct = 2 * number[0] - 1;
    }
    else if ((number[0] == 1 && number[1] == 2) || (number[0] == 2 && number[1] == 1)) {
      direct = 2;
    }
    else if ((number[0] == 2 && number[1] == 3) || (number[0] == 3 && number[1] == 2)) {
      direct = 4;
    }
    else if ((number[0] == 3 && number[1] == 4) || (number[0] == 4 && number[1] == 3)) {
      direct = 6;
    }
    else if ((number[0] == 4 && number[1] == 1) || (number[0] == 1 && number[1] == 4)) {
      direct = 8;
    } else {
      direct = 9; // なんかバグってます
    }
  } else {
    direct = 2 * number[0] - 1;
  }

  //モーター系
  if (direct == 0) {
    Serial.println("No sound detected");
    rightMotor.stop();
    leftMotor.stop();
  }
  else if (direct == 1) {
    Serial.println("front");
    rightMotor.go(255);
    leftMotor.go(255);
  }
  else if (direct == 2) {
    Serial.println("front/right");
    rightMotor.go(122.5);
    leftMotor.go(255);
  }
  else if (direct == 3) {
    Serial.println("right");
    rightMotor.stop();
    leftMotor.go(255);
  }
  else if (direct == 4) {
    Serial.println("right/back");
    rightMotor.back(122.5);
    leftMotor.back(255);
  }
  else if (direct == 5) {
    Serial.println("back");
    rightMotor.back(255);
    leftMotor.back(255);
  }
  else if (direct == 6) {
    Serial.println("left/back");
    rightMotor.back(255);
    leftMotor.back(122.5);
  }
  else if (direct == 7) {
    Serial.println("left");
    rightMotor.go(255);
    leftMotor.stop();
  }
  else if (direct == 8) {
    Serial.println("front/left");
    rightMotor.go(255);
    leftMotor.go(122.5);
  }
  else {
    Serial.println("なんかバグってますよ");
  }


  // ここからは音の高さで方向検知するやつ
  // 2290Hzから70Hz刻みでやってます。
  if (freq[0] == 33) {
    Serial.println("12時");
  }
  else if (freq[0] == 34) {
    Serial.println("1時");
  }
  else if (freq[0] == 35) {
    Serial.println("2時");
  }
  else if (freq[0] == 36) {
    Serial.println("3時");
  }
  else if (freq[0] == 37) {
    Serial.println("4時");
  }
  else if (freq[0] == 38) {
    Serial.println("5時");
  }
  else if (freq[0] == 39) {
    Serial.println("6時");
  }
  else if (freq[0] == 40) {
    Serial.println("7時");
  }
  else if (freq[0] == 41) {
    Serial.println("8時");
  }
  else if (freq[0] == 42) {
    Serial.println("9時");
  }
  else if (freq[0] == 43) {
    Serial.println("10時");
  }
  else if (freq[0] == 44) {
    Serial.println("11時");
  }

  Serial.print("音のでかさは：");
  Serial.println(vol[0]);
  Serial.print("音の高さは：");
  if (freq[0] - 33 < 0) {
    Serial.print(0);
  } else {
    Serial.print((freq[0] - 33) * 70 + 2290);
  }
  Serial.println(" Hz");
}
