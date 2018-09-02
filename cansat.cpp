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
  // setGoal(139.657881, 35.554789);  // ゴール設定(矢上グラウンド奥)
  setGoal(4014236.80, 13998730.00);  // ゴール設定(能代)

  Serial.begin(9600);

  sd.setupSd();
  Serial.println("Sd is ok");

  radio.setupRadio();
  Serial.println("Radio is ok");

  gps.setupGps();
  Serial.println("Gps is ok");

  // 水平にしてキャリブレーション
  digitalWrite(RED_LED_PIN, HIGH);
  tone(BUZZER_PIN, 131, 2000);
  acc.setupAcc();
  digitalWrite(RED_LED_PIN, LOW);
  Serial.println("Acc is ok");

  // roll,pitch,yawに回してキャリブレーション
  digitalWrite(BLUE_LED_PIN, HIGH);
  tone(BUZZER_PIN, 523, 2000);
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
  ////////////////////////////////////////////////////
//   地上局からstate変更
//  radio.getData();
//  if (laststate != radio.radio_get_data - 48) {
//  state = radio.radio_get_data - 48;
//  laststate = radio.radio_get_data - 48;
//  }
  ///////////////////////////////////////////////////
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

  //////////////////////////////////////////////////////
  // guidance1テスト
  //   // GPS無しでは停止
  //  if (gps.lat < 1 && gps.lon < 1) {
  //    leftMotor.stop();
  //    rightMotor.stop();
  //  }
  //  else {
  //    // 走行フェーズではガイダンス則に従う
  //    guidance1(gps.lon, gps.lat, compass.deg, destLon, destLat);
  //}
  //////////////////////////////////////////////////////
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
                    + String(micb.maxvol) + ", "
                    + String(direct2) + ","
                    + String(distance2) + ","
                    + String(soundvol) + ","
                    + String(millis() - guidance4Time);
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
                     + String(direct2) + ","
                     //+ String(round(15000 * 0.05 * exp(-0.05 * vol[0])))
                     + String(distance2) + ","
                     + String(soundvol) + ","
                     + String(millis() - guidance4Time) + ","
                     + "e";
  radio.sendData(send_data);
}
///////////////////////////////////////////////////////////////////////////////////
// 131 147 165 175 196 220 247
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
//    if (countPreLoop > COUNT_LIGHT1_LOOP_THRE)  state = FLYING;//通常（本番用はこっち）
        state = RUNNING;//ボイド缶検知、放出検知、着地検知、分離を省略（guidanceチェック用）

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
  //    if (droppingTime != 0) {
  //      if (millis() - landingTime > RELEASING1_TIME_THRE) digitalWrite(RELEASING1_PIN, HIGH);
  //    }
  //    if (droppingTime != 0) {
  //      if (millis() - landingTime > RELEASING1_TIME2_THRE) digitalWrite(RELEASING1_PIN, LOW);
  //    }
  // 加速度から着地検知
  if ((pow(acc.ax, 2) + pow(acc.ay, 2) + pow(acc.az, 2)) < pow(ACC_THRE, 2)) {
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
    //    tone(BUZZER_PIN, 523, 3000);
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(BLUE_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
  }
  // 着地フェーズでは第2パラシュート分離を行う
  digitalWrite(RELEASING2_PIN, HIGH);
  countReleasingLoop++;
  //  if (landingTime != 0) {
  //    if (countReleasingLoop > COUNT_RELEASING_LOOP_THRE) {
  //      digitalWrite(RELEASING2_PIN, LOW);
  //      state = RUNNING;
  //    }
  if (landingTime != 0) {
    if (millis() - landingTime > RELEASING2_TIME_THRE ) {
      state = RUNNING;
      digitalWrite(RELEASING2_PIN, LOW);
    }
  }
}

// State = 4
void Cansat::running() {
  if (runningTime == 0) {
    analogWrite(RELEASING2_PIN, 0);
    runningTime = millis();
    tone(BUZZER_PIN, 121, 2000);
    analogWrite(RELEASING2_PIN, 0);
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(BLUE_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, HIGH);
  }

  //  if (runningTime != 0) {
  //    if (millis() - runningTime < 10000) {
  //      rightMotor.go(255);
  //      leftMotor.go(255);
  //    }

  countRunning++;
  if (countRunning < 10) {
    rightMotor.go(255);
    leftMotor.go(255);
  }
  else {
    //    guidance3();
    guidance4();
  }
  // GPS無しでは停止
  //  if (gps.lat < 1 && gps.lon < 1) {
  //    leftMotor.stop();
  //    rightMotor.stop();
  //  }
  //  else {
  //    if (runningTime == 0) {
  //      runningTime = millis();
  //    }
  // 走行フェーズではガイダンス則に従う
  //    guidance1(gps.lon, gps.lat, compass.deg, destLon, destLat);
  //    if (fabs(destLon - gps.lon) <= GOAL_THRE && fabs(destLat - gps.lat) <= GOAL_THRE) state = GOAL;
  //  }
  //}
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

/////////////////////////////////////////////////////////////////////////////////////////
///**
//  @void guidance3
//  @author Tomiyoshi
//  @date Created: 20180811

void Cansat::sort(int vol[4], int freq[4], int number[4]) {
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
}

void Cansat::guidance3() {
  int vol[4] = {micf.maxvol, micr.maxvol, micb.maxvol, micl.maxvol}; // 各マイクが拾った音の大きさ
  int freq[4] = {micf.maxfreq, micr.maxfreq, micb.maxfreq, micl.maxfreq}; //各マイクが拾った音の高さ
  int number[4] = {1, 2, 3, 4};//各マイクの番号(1,2,3,4→前、右、後、左のつもり)

  // 並び替え(バブルソート)→音が大きい順にvol,freq,numberを並び替える
  sort(vol, freq, number);

  //　向き判定
  if (vol[0] < 5) {
    vol[0] = 0;
    freq[0] = 0;
    direct2 = 0;
  }
  else if (vol[0] > 5) {
    if ((float)vol[0] / (float)vol[1] > 1.3) {
      direct2 = 2 * number[0] - 1;
    }
    else if ((number[0] == 1 && number[1] == 2) || (number[0] == 2 && number[1] == 1)) {
      direct2 = 2;
    }
    else if ((number[0] == 2 && number[1] == 3) || (number[0] == 3 && number[1] == 2)) {
      direct2 = 4;
    }
    else if ((number[0] == 3 && number[1] == 4) || (number[0] == 4 && number[1] == 3)) {
      direct2 = 6;
    }
    else if ((number[0] == 4 && number[1] == 1) || (number[0] == 1 && number[1] == 4)) {
      direct2 = 8;
    } else {
      direct2 = 9; // なんかバグってます
    }
  } else {
    direct2 = 2 * number[0] - 1;
  }

  //モーター系
  if (direct2 == 0) {
    Serial.println("No sound detected");
    rightMotor.stop();
    leftMotor.stop();
  }
  else if (direct2 == 1) {
    Serial.println("front");
    rightMotor.go(255);
    leftMotor.go(255);
  }
  else if (direct2 == 2) {
    Serial.println("front/right");
    rightMotor.go(122.5);
    leftMotor.go(255);
  }
  else if (direct2 == 3) {
    Serial.println("right");
    rightMotor.stop();
    leftMotor.go(255);
  }
  else if (direct == 4) {
    Serial.println("right/back");
    //  rightMotor.back(122.5);
    rightMotor.stop();
    leftMotor.go(255);
  }
  else if (direct2 == 5) {
    Serial.println("back");
    // rightMotor.back(255);
    rightMotor.stop();
    leftMotor.go(255);
  }
  else if (direct2 == 6) {
    Serial.println("left/back");
    rightMotor.go(255);
    leftMotor.stop();
    //  leftMotor.back(122.5);
  }
  else if (direct2 == 7) {
    Serial.println("left");
    rightMotor.go(255);
    leftMotor.stop();
  }
  else if (direct2 == 8) {
    Serial.println("front/left");
    rightMotor.go(255);
    leftMotor.go(122.5);
  }
  else {
    Serial.println("なんかバグってますよ");
    rightMotor.stop();
    leftMotor.stop();

  }
  distance2 = round(15000 * 0.05 * exp(-0.05 * vol[0]));
  if (vol[0] > 70) state = GOAL;
}

/////////////////////////////////////////////////////////////////////////////////////////
///**
//  @void guidance4
//  @author Tomiyoshi
//  @date Created: 20180812

void Cansat::guidance4running(float nowDeg, float directDeg) {

  if (nowDeg < 0) {
    directAngle = directDeg - (nowDeg + 360);
  } else {
    directAngle = directDeg - nowDeg;
  }

  if (directAngle < -180) {
    directAngle = directAngle + 360;
  }

  if (directAngle > 180) {
    directAngle = directAngle - 360;
  }

  if (directAngle < (ANGLE_THRE - 10) && directAngle > -(ANGLE_THRE - 10)) {
    direct = 0; //真っ直ぐ
  } else {
    if (directAngle >= 0) {
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
    rightMotor.go(255 * (1 - directAngle / 180));
    leftMotor.go(255);
  } else if (direct == -1) { //左
    rightMotor.go(255);
    leftMotor.go(255 * (1 + directAngle / 180));
  }
}

void Cansat::guidance4() {
  // ループ開始時刻保存
  if (guidance4Time == 0) {
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(BLUE_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, HIGH);
    rightMotor.stop();
    leftMotor.stop();
    guidance4Time = millis();
  }
  if (guidance4Time != 0) {
    int vol[4] = {micf.maxvol, micr.maxvol, micb.maxvol, micl.maxvol}; // 各マイクが拾った音の大きさ
    int freq[4] = {micf.maxfreq, micr.maxfreq, micb.maxfreq, micl.maxfreq}; //各マイクが拾った音の高さ
    int number[4] = {1, 2, 3, 4};//各マイクの番号(1,2,3,4→前、右、後、左のつもり)

    sort(vol, freq, number);

    // ゴール判定は毎ループやる
    if (vol[0] > 60)state = GOAL;//ここのifの条件式の数字をいじることで閾値を変更可能
    // if (distance2 < 50 && distance2 > 0)state = GOAL;

    //    unsigned long GUIDANCE4_TIME_THRE2 = 40000;
    unsigned long GUIDANCE4_TIME_THRE2 = 27000;//テスト用

    if (millis() - guidance4Time < GUIDANCE4_TIME_THRE) {
      // 一定時間停止し、どの高さの音が一番大きく聞こえたかを判定

      if (freq[0] < N + 1 || N + 8 < freq[0]) {
        vol[0] = 0;
      }

      if (vol[0] < 5) {
        vol[0] = 0;
        freq[0] = 0;
      }
      int distance_candidate = 0;
      if (vol[0] > 0) {
        switch (freq[0]) {
          case N+1:
            distance_candidate = round(-159.8 * log(vol[0]) + 631.1);
            break;
          case N+2:
            distance_candidate = round(-161.6 * log(vol[0]) + 637.61);
            break;
          case N+3:
            distance_candidate = round(-160.2 * log(vol[0]) + 628.52);
            break;
          case N+4:
            distance_candidate = round(-148.1 * log(vol[0]) + 585.04);
            break;
          case N+5:
            distance_candidate = round(-143.1 * log(vol[0]) + 562.17);
            break;
          case N+6:
            distance_candidate = round(-131.6 * log(vol[0]) + 513.86);
            break;
          case N+7:
            distance_candidate = round(-123.2 * log(vol[0]) + 472.33);
            break;
          case N+8:
            distance_candidate = round(-106.9 * log(vol[0]) + 417.9);
            break;
        }
      }
      if(distance2==0&&distance_candidate!=0){
        soundvol = vol[0];
        soundfreq = freq[0];
        distance2 = distance_candidate;
      }
      else if (distance2 > distance_candidate && distance_candidate != 0) {
        soundvol = vol[0];
        soundfreq = freq[0];
        distance2 = distance_candidate;
      }
    }
    else if (millis() - guidance4Time < GUIDANCE4_TIME_THRE2) {
      // 向き、距離決定→走行
      if (countGuidance4Loop == 0) {
        digitalWrite(RED_LED_PIN, LOW);
        digitalWrite(BLUE_LED_PIN, LOW);
        digitalWrite(GREEN_LED_PIN, LOW);
        // distance2 = round(15000 * 0.05 * exp(-0.05 * soundvol));
        direct2 = soundfreq - N; // Nはこちらが任意で決める値
        if (distance2 == 0) {
          guidance4Time = 0;
        } else {
          countGuidance4Loop++;
        }
      }
      else {
        directDeg = 45 * (direct2 - 1); // 8方向検知、0は真北？
        //        switch (direct2) {
        //          case 1:
        //            break;
        //          case 2:
        //            break;
        //          case 3:
        //            break;
        //          case 4:
        //            break;
        //          case 5:
        //            break;
        //          case 6:
        //            break;
        //          case 7:
        //            break;
        //          case 8:
        //            break;
        //        }
        // ここに引数を「機体の現在の向き、機体の向かいたい向き」としたモータ制御関数を入れる
        // guidance1()を参考に書く
        // soundvolの大きさによって速度を変えるのも面白いかも
        guidance4running(compass.deg, directDeg);
      }
    }

    if (millis() - guidance4Time > GUIDANCE4_TIME_THRE2) {
      // ここまでで1セット、各数値を初期化
      guidance4Time = 0;
      countGuidance4Loop = 0;
      soundvol = 0;
      soundfreq = 0;
      direct2 = 0;
      distance2 = 0;
      rightMotor.stop();
      leftMotor.stop();
    }
  }
}

// State = 5
void Cansat::goal() {
  // stopslowlyの同時駆動即が必要
  leftMotor.stop();
  rightMotor.stop();
  digitalWrite(RED_LED_PIN, HIGH); delay(100);
  digitalWrite(BLUE_LED_PIN, HIGH); delay(100);
  digitalWrite(GREEN_LED_PIN, HIGH); delay(100);
  digitalWrite(RED_LED_PIN, LOW); delay(100);
  digitalWrite(BLUE_LED_PIN, LOW); delay(100);
  digitalWrite(GREEN_LED_PIN, LOW); delay(100);
}
///////////////////////////////////////////////////////////////////////////////////
