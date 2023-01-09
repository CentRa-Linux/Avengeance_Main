// Avengeance's main program for ESP32

// i2c Adresses
// BMX055: ACCELEROMETER:0x19 GYRO:0x69 MAGNETIC:0x13
// TCA9548A: 0x70
// S11059-02DT: 0x2A
// TPR-105F: IO25, 26, 27, 32, 33, 34, 35

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>
#include <utility/imumaths.h>

#define TEST                                                                   \
  false //競技時は此処がfalseになっていることを「必ず」確認！！！！！！

// BMX055 加速度センサのI2Cアドレス
#define Addr_Accl 0x19 // (JP1,JP2,JP3 = Openの時)
// BMX055 ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69 // (JP1,JP2,JP3 = Openの時)
// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13 // (JP1,JP2,JP3 = Openの時)
// 加速度、地磁気センサのオフセット
#define c_a_x 0.685
#define c_a_y -0.87
#define c_a_z -0.15
#define c_m_x 108
#define c_m_y -10
#define c_m_z 0

// S11059のアドレス、定数
#define S11059_ADDR 0x2A
#define CONTROL_MSB 0x00
#define CONTROL_1_LSB 0x89
#define CONTROL_2_LSB 0x09
#define SENSOR_REGISTER 0x03
#define MONO_0 27
#define MONO_1 26
#define MONO_2 25
#define MONO_3 33
#define MONO_4 32
#define MONO_5 35
#define MONO_6 34
//超音波のピン
#define SS_F_T 18
#define SS_F_E 5
// モータードライバのピン
#define PWMA 17 // 左
#define PWMB 16 // 右
#define AIN1 4
#define BIN1 2
#define AIN2 0
#define BIN2 15
// ライントレース系の定数
#define lmax 610
#define lmin 117
#define rmax 497
#define rmin 80
#define Kp 80
#define Kp2 30

#define rc_c_v 0.5

#define lr_max 1480
#define lg_max 1910
#define lb_max 1435
#define rr_max 1300
#define rg_max 1720
#define rb_max 1275
#define lr_min 612
#define lg_min 730
#define lb_min 548
#define rr_min 555
#define rg_min 692
#define rb_min 507

#define th_lw_v 20 // 白だと判断する明度
#define th_rw_v 20

#define th_lg_h 129
#define th_rg_h 126
#define th_lg_s 200
#define th_rg_s 200

#define mono0 1
#define mono1 30
#define mono2 250
#define mono3 250
#define mono4 250
#define mono5 90
#define mono6 50

// Tof センサのアドレス
#define ADDRESS 0x52

float roll = 0.0; //機体のroll角

float pitch = 0.0; //機体のpitch角

float yaw = 0.0; //機体のyaw角

// センサーの色の値を保存するグローバル変数
int rColor[4];
float rpRGB[3];
float rRGB[3];
float rHSV[3];
int lColor[4];
float lpRGB[3];
float lRGB[3];
float lHSV[3];
int mono[7];
int sensors[9];

//ライントレース用の変数
int point = 0;

// 前後左右のTofセンサの値を格納する場所　0=前 1=後 2=左 3=右
uint16_t distance[4];

// I2Cデバイスのアドレスをチェック and correct line below (by default address is
// 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

// float tan2angle(float x, float y) { return atan2(y, x); }

// TCA9548Aのi2cをスイッチする関数
void SWITCH_BUS_TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();
}

// BNO055でオイラー角を算出する関数
void get_angle() {
  SWITCH_BUS_TCA9548A(0);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  // roll pitch yawに機体の角度を保存
  roll = euler.x() - 180;
  pitch = euler.y();
  yaw = euler.z(); // yawの値の範囲は　-180 < yaw < 180
}

void init_color() {
  SWITCH_BUS_TCA9548A(0);
  Wire.beginTransmission(
      S11059_ADDR); // I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(CONTROL_MSB);   //コントロールバイトを指定
  Wire.write(CONTROL_1_LSB); // ADCリセット、スリープ解除
  Wire.endTransmission(); // I2Cスレーブ「Arduino Uno」のデータ送信終了

  Wire.beginTransmission(
      S11059_ADDR); // I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(CONTROL_MSB);   //コントロールバイトを指定
  Wire.write(CONTROL_2_LSB); // ADCリセット解除、バスリリース
  Wire.endTransmission(); // I2Cスレーブ「Arduino Uno」のデータ送信終了

  SWITCH_BUS_TCA9548A(1);
  Wire.beginTransmission(
      S11059_ADDR); // I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(CONTROL_MSB);   //コントロールバイトを指定
  Wire.write(CONTROL_1_LSB); // ADCリセット、スリープ解除
  Wire.endTransmission(); // I2Cスレーブ「Arduino Uno」のデータ送信終了

  Wire.beginTransmission(
      S11059_ADDR); // I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(CONTROL_MSB);   //コントロールバイトを指定
  Wire.write(CONTROL_2_LSB); // ADCリセット解除、バスリリース
  Wire.endTransmission(); // I2Cスレーブ「Arduino Uno」のデータ送信終了
}

void get_color() {
  int high_byte, low_byte;

  SWITCH_BUS_TCA9548A(0);
  Wire.beginTransmission(
      S11059_ADDR); // I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(SENSOR_REGISTER); //出力データバイトを指定
  Wire.endTransmission(); // I2Cスレーブ「Arduino Uno」のデータ送信終了

  Wire.requestFrom(S11059_ADDR,
                   8); // I2Cデバイス「S11059_ADDR」に8Byteのデータ要求
  if (Wire.available()) {
    high_byte = Wire.read(); // high_byteに「赤(上位バイト)」のデータ読み込み
    low_byte = Wire.read(); // high_byteに「赤(下位バイト)」のデータ読み込み
    rColor[0] =
        high_byte << 8 |
        low_byte; // 1Byte目のデータを8bit左にシフト、OR演算子で2Byte目のデータを結合して、redに代入

    high_byte = Wire.read(); // high_byteに「緑(上位バイト)」のデータ読み込み
    low_byte = Wire.read(); // high_byteに「緑(下位バイト)」のデータ読み込み
    rColor[1] =
        high_byte << 8 |
        low_byte; // 1Byte目のデータを8bit左にシフト、OR演算子で2Byte目のデータを結合して、greenに代入

    high_byte = Wire.read(); // high_byteに「青(上位バイト)」のデータ読み込み
    low_byte = Wire.read(); // high_byteに「青(下位バイト)」のデータ読み込み
    rColor[2] =
        high_byte << 8 |
        low_byte; // 1Byte目のデータを8bit左にシフト、OR演算子で2Byte目のデータを結合して、blueに代入

    high_byte = Wire.read(); // high_byteに「赤外(上位バイト)」のデータ読み込み
    low_byte = Wire.read(); // high_byteに「赤外(下位バイト)」のデータ読み込み
    rColor[3] =
        high_byte << 8 |
        low_byte; // 1Byte目のデータを8bit左にシフト、OR演算子で2Byte目のデータを結合して、IRに代入
  }
  Wire.endTransmission(); // I2Cスレーブ「Arduino Uno」のデータ送信終了

  SWITCH_BUS_TCA9548A(1);
  Wire.beginTransmission(
      S11059_ADDR); // I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(SENSOR_REGISTER); //出力データバイトを指定
  Wire.endTransmission(); // I2Cスレーブ「Arduino Uno」のデータ送信終了

  Wire.requestFrom(S11059_ADDR,
                   8); // I2Cデバイス「S11059_ADDR」に8Byteのデータ要求
  if (Wire.available()) {
    high_byte = Wire.read(); // high_byteに「赤(上位バイト)」のデータ読み込み
    low_byte = Wire.read(); // high_byteに「赤(下位バイト)」のデータ読み込み
    lColor[0] =
        high_byte << 8 |
        low_byte; // 1Byte目のデータを8bit左にシフト、OR演算子で2Byte目のデータを結合して、redに代入

    high_byte = Wire.read(); // high_byteに「緑(上位バイト)」のデータ読み込み
    low_byte = Wire.read(); // high_byteに「緑(下位バイト)」のデータ読み込み
    lColor[1] =
        high_byte << 8 |
        low_byte; // 1Byte目のデータを8bit左にシフト、OR演算子で2Byte目のデータを結合して、greenに代入

    high_byte = Wire.read(); // high_byteに「青(上位バイト)」のデータ読み込み
    low_byte = Wire.read(); // high_byteに「青(下位バイト)」のデータ読み込み
    lColor[2] =
        high_byte << 8 |
        low_byte; // 1Byte目のデータを8bit左にシフト、OR演算子で2Byte目のデータを結合して、blueに代入

    high_byte = Wire.read(); // high_byteに「赤外(上位バイト)」のデータ読み込み
    low_byte = Wire.read(); // high_byteに「赤外(下位バイト)」のデータ読み込み
    lColor[3] =
        high_byte << 8 |
        low_byte; // 1Byte目のデータを8bit左にシフト、OR演算子で2Byte目のデータを結合して、IRに代入
  }
  Wire.endTransmission(); // I2Cスレーブ「Arduino Uno」のデータ送信終了
}

void calculate_color() {
  rpRGB[0] = rRGB[0];
  rpRGB[1] = rRGB[1];
  rpRGB[2] = rRGB[2];
  lpRGB[0] = lRGB[0];
  lpRGB[1] = lRGB[1];
  lpRGB[2] = lRGB[2];
  rRGB[0] = rc_c_v * rpRGB[0] +
            (1 - rc_c_v) * (rColor[0] - rr_min) * 255.0 / (rr_max - rr_min);
  rRGB[1] = rc_c_v * rpRGB[1] +
            (1 - rc_c_v) * (rColor[1] - rg_min) * 255.0 / (rg_max - rg_min);
  rRGB[2] = rc_c_v * rpRGB[2] +
            (1 - rc_c_v) * (rColor[2] - rb_min) * 255.0 / (rb_max - rb_min);
  lRGB[0] = rc_c_v * lpRGB[0] +
            (1 - rc_c_v) * (lColor[0] - lr_min) * 255.0 / (lr_max - lr_min);
  lRGB[1] = rc_c_v * lpRGB[1] +
            (1 - rc_c_v) * (lColor[1] - lg_min) * 255.0 / (lg_max - lg_min);
  lRGB[2] = rc_c_v * lpRGB[2] +
            (1 - rc_c_v) * (lColor[2] - lb_min) * 255.0 / (lb_max - lb_min);
  if (max(max(rRGB[0], rRGB[1]), rRGB[2]) == rRGB[0]) {
    rHSV[0] = 60.0 * (rRGB[1] - rRGB[2]) /
              (max(max(rRGB[0], rRGB[1]), rRGB[2]) -
               min(min(rRGB[0], rRGB[1]), rRGB[2]));
  }
  if (max(max(rRGB[0], rRGB[1]), rRGB[2]) == rRGB[1]) {
    rHSV[0] = 60.0 * (rRGB[2] - rRGB[0]) /
                  (max(max(rRGB[0], rRGB[1]), rRGB[2]) -
                   min(min(rRGB[0], rRGB[1]), rRGB[2])) +
              120.0;
  }
  if (max(max(rRGB[0], rRGB[1]), rRGB[2]) == rRGB[2]) {
    rHSV[0] = 60.0 * (rRGB[0] - rRGB[1]) /
                  (max(max(rRGB[0], rRGB[1]), rRGB[2]) -
                   min(min(rRGB[0], rRGB[1]), rRGB[2])) +
              240.0;
  }
  rHSV[0] = abs(rHSV[0]);
  rHSV[1] = abs((max(max(rRGB[0], rRGB[1]), rRGB[2]) -
                 min(min(rRGB[0], rRGB[1]), rRGB[2])) /
                max(max(rRGB[0], rRGB[1]), rRGB[2])) *
            255.0;
  rHSV[2] = max(max(rRGB[0], rRGB[1]), rRGB[2]);

  if (max(max(lRGB[0], lRGB[1]), lRGB[2]) == lRGB[0]) {
    lHSV[0] = 60.0 * (lRGB[1] - lRGB[2]) /
              (max(max(lRGB[0], lRGB[1]), lRGB[2]) -
               min(min(lRGB[0], lRGB[1]), lRGB[2]));
  }
  if (max(max(lRGB[0], lRGB[1]), lRGB[2]) == lRGB[1]) {
    lHSV[0] = 60.0 * (lRGB[2] - lRGB[0]) /
                  (max(max(lRGB[0], lRGB[1]), lRGB[2]) -
                   min(min(lRGB[0], lRGB[1]), lRGB[2])) +
              120.0;
  }
  if (max(max(lRGB[0], lRGB[1]), lRGB[2]) == lRGB[2]) {
    lHSV[0] = 60.0 * (lRGB[0] - lRGB[1]) /
                  (max(max(lRGB[0], lRGB[1]), lRGB[2]) -
                   min(min(lRGB[0], lRGB[1]), lRGB[2])) +
              240.0;
  }
  lHSV[0] = abs(lHSV[0]);
  lHSV[1] = abs((max(max(lRGB[0], lRGB[1]), lRGB[2]) -
                 min(min(lRGB[0], lRGB[1]), lRGB[2])) /
                max(max(lRGB[0], lRGB[1]), lRGB[2])) *
            255.0;
  lHSV[2] = max(max(lRGB[0], lRGB[1]), lRGB[2]);

  if (lHSV[2] > th_lw_v) {
    sensors[6] = 1;
    if (lHSV[2] > 400) {
      sensors[6] = 4;
    }
  } else {
    sensors[6] = 0;
  }
  if (rHSV[2] > th_rw_v) {
    if (rHSV[2] > 400) {
      sensors[2] = 4;
    }
    sensors[2] = 1;
  } else {
    sensors[2] = 0;
  }
  if (lRGB[0] - lRGB[1] > 60 && (lRGB[0] + lRGB[1] + lRGB[2]) / 3 < 150) {
    sensors[6] = 3;
  }
  if (rRGB[0] - rRGB[1] > 60 && (rRGB[0] + rRGB[1] + rRGB[2]) / 3 < 150) {
    sensors[2] = 3;
  }
  if (lRGB[0] - lRGB[1] < -25 && (lRGB[0] + lRGB[1] + lRGB[2]) / 3 < 90) {
    sensors[6] = 2;
  }
  if (rRGB[0] - rRGB[1] < -25 && (rRGB[0] + rRGB[1] + rRGB[2]) / 3 < 90) {
    sensors[2] = 2;
  }
}

void get_mono() {
  mono[0] = analogRead(MONO_0);
  mono[1] = analogRead(MONO_1);
  mono[2] = analogRead(MONO_2);
  mono[3] = analogRead(MONO_3);
  mono[4] = analogRead(MONO_4);
  mono[5] = analogRead(MONO_5);
  mono[6] = analogRead(MONO_6);
  if (mono[0] < mono0) {
    sensors[0] = 0;
  } else {
    sensors[0] = 1;
  }
  if (mono[1] < mono1) {
    sensors[1] = 0;
  } else {
    sensors[1] = 1;
  }
  if (mono[2] < mono2) {
    sensors[3] = 0;
  } else {
    sensors[3] = 1;
  }
  if (mono[3] < mono3) {
    sensors[4] = 0;
  } else {
    sensors[4] = 1;
  }
  if (mono[4] < mono4) {
    sensors[5] = 0;
  } else {
    sensors[5] = 1;
  }
  if (mono[5] < mono5) {
    sensors[7] = 0;
  } else {
    sensors[7] = 1;
  }
  if (mono[6] < mono6) {
    sensors[8] = 0;
  } else {
    sensors[8] = 1;
  }
}

void init_motor() {
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN2, OUTPUT);
  ledcSetup(0, 50000, 8);
  ledcSetup(1, 50000, 8);
  ledcAttachPin(PWMA, 0);
  ledcAttachPin(PWMB, 1);
}

void drive_motor(int a, int b) {
  if (TEST == true) {
    Serial.print(a);
    Serial.print(",");
    Serial.println(b);
    return;
  }
  if (a == 0) {
    // BRAKE
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
    ledcWrite(0, a);
  }
  if (a > 0) {
    // CW
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(0, a);
  }
  if (a < 0) {
    // CCW
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(0, abs(a));
  }
  if (b == 0) {
    // BRAKE
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
    ledcWrite(1, b);
  }
  if (b > 0) {
    // CW
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(1, b);
  }
  if (b < 0) {
    // CCW
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(1, abs(b));
  }
}

//目標旋回角度に到達したか確認する関数
bool decide(float angle, float target, float origin,
            float acceptable_error) { /*
Serial.print(abs(target - angle));
Serial.print(",");
Serial.println(abs(origin - angle));*/
  if (angle > 0) {
    if (abs(target - angle) < acceptable_error ||
        abs(origin - angle) < acceptable_error) {
      Serial.println("end rotating using acc. sensor");
      return false;
    } else {
      return true;
    }
  } else if (angle < 0) {
    if (abs(target - angle) < acceptable_error ||
        abs(origin - angle) < acceptable_error) {
      Serial.println("end rotating using acc. sensor");
      return false;
    } else {
      return true;
    }
  }
}

void debug_mono() {
  for (int i = 8; i >= 0; i--) {
    Serial.print(sensors[i]);
    Serial.print(",");
  }
  Serial.println("");
}

void setled(int i1, int i2, int i3) {
  Wire.beginTransmission(8);
  Wire.write(i1);
  Wire.write(i2);
  Wire.write(i3);
  Wire.endTransmission();
}

//旋回を行う関数 float angle の値は-180以上180以下
void rotate_NC(int angle, int error) {
  get_angle();
  //旋回前のオイラー角
  float pres_angle = roll;
  //旋回目的のオイラー角（定義域範囲外の値もとりうる）
  float target = roll + angle;
  float origin = target;
  if (angle > 0) {
    if (target > 180)
      target -= 360;
  } else if (angle < 0) {
    if (target < -180)
      target += 360;
  }
  Serial.println("start rotating");
  if (angle > 0) {
    drive_motor(100, -100);
    delay(100);
  } else if (angle < 0) {
    drive_motor(-100, 100);
    delay(100);
  }
  while (decide(roll, target, origin, error)) {
    get_angle();
    if (angle > 0) {
      drive_motor(60, -60);
    } else if (angle < 0) {
      drive_motor(-60, 60);
    }
    get_color();
    calculate_color();
    get_mono();
    setled(3, 3, 0);
  }
  Serial.println("end");
  drive_motor(0, 0);
  delay(100);
}

void p_slowtrace() {
  setled(2, 3, 3);
  float value =
      ((mono[2] - lmax) * 2.0 / lmax + 1) - ((mono[4] - rmax) * 2.0 / rmax + 1);
  if ((sensors[0] == 0 || sensors[1] == 0) &&
      (sensors[7] == 1 || sensors[8] == 1)) {
    value = -1.5;
    point -= 50;
  }
  if ((sensors[0] == 1 || sensors[1] == 1) &&
      (sensors[7] == 0 || sensors[8] == 0)) {
    value = 1.5;
    point += 50;
  }
  if ((sensors[0] == 0 || sensors[1] == 0) &&
      (sensors[7] == 0 || sensors[8] == 0)) {
    value = 0;
    if (point > 1000) {
      value = -1.0;
      point = 2000;
      setled(3, 0, 3);
    }
    if (point < -1000) {
      value = 1.0;
      point = -2000;
      setled(3, 3, 0);
    }
  }
  int l = int(65.0 - value * Kp2);
  int r = int(65.0 + value * Kp2);
  if (l > 65) {
    l = 65;
  }
  if (r > 65) {
    r = 65;
  }
  drive_motor(l, r);
}

void p_linetrace() {
  setled(2, 3, 3);
  float value =
      ((mono[2] - lmax) * 2.0 / lmax + 1) - ((mono[4] - rmax) * 2.0 / rmax + 1);
  if ((sensors[0] == 0 || sensors[1] == 0) &&
      (sensors[7] == 1 || sensors[8] == 1)) {
    value = -1.5;
    point -= 50;
  }
  if ((sensors[0] == 1 || sensors[1] == 1) &&
      (sensors[7] == 0 || sensors[8] == 0)) {
    value = 1.5;
    point += 50;
  }
  if ((sensors[0] == 0 || sensors[1] == 0) &&
      (sensors[7] == 0 || sensors[8] == 0)) {
    value = 0;
  }
  int l = int(65.0 - value * Kp);
  int r = int(65.0 + value * Kp);
  if (l > 65) {
    l = 65;
  }
  if (r > 65) {
    r = 65;
  }
  drive_motor(l, r);
}

void escape() {
  if (sensors[2] == 4 && sensors[6] == 4) {
    int now = millis() + 1500;
    while (millis() < now) {
      get_color();
      calculate_color();
      get_mono();
      drive_motor(70, 70);
    }
    rotate_NC(PI / 2, 15);
    while (distance[0] > 150) {
      drive_motor(70, 70);
    }
    rotate_NC(PI / 2, 15);
    while (sensors[3] == 1 && sensors[4] == 1 && sensors[5] == 1) {
      get_color();
      calculate_color();
      get_mono();
      p_linetrace();
    }
  }
}

void judge_obstacle() {
  if (distance[1] < 150 && distance[1] != 8888) {
    Serial.println("found obstacle");
    int now = millis() + 500;
    while (millis() < now) {
      get_color();
      calculate_color();
      get_mono();
      drive_motor(-70, -70);
    }
    rotate_NC(90, 15);
    while (sensors[3] == 1 && sensors[4] == 1 && sensors[5] == 1) {
      get_mono();
      drive_motor(120, 40);
    }
    rotate_NC(90, 15);
    drive_motor(0, 0);
  }
}

void judge_gap() {
  if (sensors[0] == 1 && sensors[1] && sensors[3] == 1 && sensors[4] == 1 &&
      sensors[5] == 1 && sensors[7] == 1 && sensors[8] == 1) {
    Serial.println("Found gap");
    get_color();
    calculate_color();
    get_mono();
    while (sensors[0] == 1 && sensors[1] && sensors[3] == 1 &&
           sensors[4] == 1 && sensors[5] == 1 && sensors[7] == 1 &&
           sensors[8] == 1) {
      get_color();
      calculate_color();
      get_mono();
      int now = millis() + 500;
      while (sensors[0] == 1 && sensors[1] && sensors[3] == 1 &&
             sensors[4] == 1 && sensors[5] == 1 && sensors[7] == 1 &&
             sensors[8] == 1 && millis() < now) {
        get_color();
        calculate_color();
        get_mono();
        drive_motor(70, 70);
      }
      now = millis();
      while (sensors[0] == 1 && sensors[1] && sensors[3] == 1 &&
             sensors[4] == 1 && sensors[5] == 1 && sensors[7] == 1 &&
             sensors[8] == 1 && millis() < now + 1500) {
        get_color();
        calculate_color();
        get_mono();
        drive_motor(70, 0);
      }
      now = millis();
      while (sensors[0] == 1 && sensors[1] && sensors[3] == 1 &&
             sensors[4] == 1 && sensors[5] == 1 && sensors[7] == 1 &&
             sensors[8] == 1 && millis() < now + 1500) {
        get_color();
        calculate_color();
        get_mono();
        drive_motor(-70, 0);
      }
      now = millis();
      while (sensors[0] == 1 && sensors[1] && sensors[3] == 1 &&
             sensors[4] == 1 && sensors[5] == 1 && sensors[7] == 1 &&
             sensors[8] == 1 && millis() < now + 1500) {
        get_color();
        calculate_color();
        get_mono();
        drive_motor(0, 70);
      }
      now = millis();
      while (sensors[0] == 1 && sensors[1] && sensors[3] == 1 &&
             sensors[4] == 1 && sensors[5] == 1 && sensors[7] == 1 &&
             sensors[8] == 1 && millis() < now + 1500) {
        get_color();
        calculate_color();
        get_mono();
        drive_motor(0, -70);
      }
      now = millis();
      while (sensors[0] == 1 && sensors[1] && sensors[3] == 1 &&
             sensors[4] == 1 && sensors[5] == 1 && sensors[7] == 1 &&
             sensors[8] == 1 && millis() < now + 1500) {
        get_color();
        calculate_color();
        get_mono();
        drive_motor(70, 70);
      }
    }
  }
}

void judge_green() {
  if (sensors[2] == 2 || sensors[6] == 2) {
    drive_motor(0, 0);
    delay(100);
    drive_motor(-100, -100);
    delay(50);
    int now = millis();
    bool l = false;
    bool r = false;
    bool checkline = false;
    if (sensors[2] == 2) {
      r = true;
    }
    if (sensors[6] == 2) {
      l = true;
    }
    Serial.println("start");
    while ((millis() < now + 2000) && (((sensors[0] == 1 || sensors[1] == 1) &&
                                        (sensors[7] == 1 || sensors[8] == 1)) ||
                                       sensors[4] == 1)) {
      get_color();
      calculate_color();
      get_mono();
      drive_motor(40, 40);
      if (sensors[2] == 2) {
        r = true;
      }
      if (sensors[6] == 2) {
        l = true;
      }
      Serial.println(sensors[4]);
      if (((sensors[0] == 0 && sensors[1] == 0) ||
           (sensors[7] == 0 && sensors[8] == 0)) &&
          sensors[4] == 0) {
        checkline = true;
      }
    }
    if (checkline == false) {
      return;
    }
    now = millis();
    if (l == true || r == true) {
      while (millis() < now + 650) {
        drive_motor(60, 60);
      }
    }
    if (l == true && r == false) {
      rotate_NC(90, 15);
    }
    if (l == false && r == true) {
      rotate_NC(-90, 15);
    }
    if (l == true && r == true) {
      rotate_NC(180, 15);
    }
    Serial.print(checkline);
    Serial.print(",");
    Serial.print(l);
    Serial.print(",");
    Serial.println(r);
  }
}

void get_distance() {
  SWITCH_BUS_TCA9548A(0);
  Wire.beginTransmission(ADDRESS);
  Wire.write(0xD3);
  Wire.endTransmission(false);
  Wire.requestFrom(ADDRESS, 2);
  int data_cnt = 0;
  distance[0] = 0;
  int distance_tmp = 0;
  while (Wire.available()) {
    distance_tmp = Wire.read();
    distance[0] = (distance[0] << (data_cnt * 8)) | distance_tmp;
    data_cnt++;
  }

  SWITCH_BUS_TCA9548A(1);
  Wire.beginTransmission(ADDRESS);
  Wire.write(0xD3);
  Wire.endTransmission(false);
  Wire.requestFrom(ADDRESS, 2);
  data_cnt = 0;
  distance[1] = 0;
  distance_tmp = 0;
  while (Wire.available()) {
    distance_tmp = Wire.read();
    distance[1] = (distance[1] << (data_cnt * 8)) | distance_tmp;
    data_cnt++;
  }

  SWITCH_BUS_TCA9548A(2);
  Wire.beginTransmission(ADDRESS);
  Wire.write(0xD3);
  Wire.endTransmission(false);
  Wire.requestFrom(ADDRESS, 2);
  data_cnt = 0;
  distance[2] = 0;
  distance_tmp = 0;
  while (Wire.available()) {
    distance_tmp = Wire.read();
    distance[2] = (distance[2] << (data_cnt * 8)) | distance_tmp;
    data_cnt++;
  }

  SWITCH_BUS_TCA9548A(3);
  Wire.beginTransmission(ADDRESS);
  Wire.write(0xD3);
  Wire.endTransmission(false);
  Wire.requestFrom(ADDRESS, 2);
  data_cnt = 0;
  distance[3] = 0;
  distance_tmp = 0;
  while (Wire.available()) {
    distance_tmp = Wire.read();
    distance[3] = (distance[3] << (data_cnt * 8)) | distance_tmp;
    data_cnt++;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("This is ESP32, Sending message to PC on Serial Connection.");

  Serial.println("Setting up i2c...");
  Wire.begin();
  Wire.setClock(400000);
  bno.begin();
  init_color();
  Serial.println("i2c Ok!");
  init_motor();
}

int now = 0;

void loop() {
  get_angle();
  get_distance();
  Serial.println(distance[1]);
  get_color();
  calculate_color();
  get_mono();
  // debug_mono();
  //  check_bmx();
  // judge_obstacle();
  judge_gap();
  judge_green();
  if (yaw > 15) {
    p_slowtrace();
  } else {
    p_linetrace();
  }
  if (point > 0) {
    point -= 5;
  }
  if (point < 0) {
    point += 5;
  }
  if (sensors[2] == 3 || sensors[6] == 3) {
    Serial.println("red");
    drive_motor(0, 0);
    while (true) {
      delay(1);
    }
  }
  /*
  // Serial.println(dist);
  // debug_mono();
  Serial.print(lHSV[0]);
  Serial.print(",");
  Serial.print(lHSV[1]);
  Serial.print(",");
  Serial.print(lHSV[2]);
  Serial.print(",");
  Serial.print(rHSV[0]);
  Serial.print(",");
  Serial.print(rHSV[1]);
  Serial.print(",");
  Serial.print(rHSV[2]);
  Serial.println(",");
  Serial.print(lRGB[0]);
  Serial.print(",");
  Serial.print(lRGB[1]);
  Serial.print(",");
  Serial.print(lRGB[2]);
  Serial.print(",");
  Serial.print(lColor[3]);
  Serial.print(",");
  Serial.print(rRGB[0]);
  Serial.print(",");
  Serial.print(rRGB[1]);
  Serial.print(",");
  Serial.print(rRGB[2]);
  Serial.print(",");
  Serial.println(rColor[3]);
  Serial.print(distance[0]);
  Serial.print(",");
  Serial.print(distance[1]);
  Serial.print(",");
  Serial.print(distance[2]);
  Serial.print(",");
  Serial.println(distance[3]);
  */
}