// Avengeance's main program for ESP32

// i2c Adresses
// BMX055: ACCELEROMETER:0x19 GYRO:0x69 MAGNETIC:0x13
// TCA9548A: 0x70
// S11059-02DT: 0x2A
// TPR-105F: IO25, 26, 27, 32, 33, 34, 35

#include <Arduino.h>
#include <Wire.h>

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

// センサーの値を保存するグローバル変数
float xAccl = 0.00;
float yAccl = 0.00;
float zAccl = 0.00;
float xCalibAccl = 0.0;
float yCalibAccl = 0.0;
float zCalibAccl = 0.0;
float xGyro = 0.00;
float yGyro = 0.00;
float zGyro = 0.00;
// 機体の角度を保存するグローバル変数
float CompVecSize = 0.00;
float bodyrad = 0.00;
float bodyasin = 0.00;
int xMag = 0;
int yMag = 0;
int zMag = 0;
int xCalibMag = 0;
int yCalibMag = 0;
int zCalibMag = 0;
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;
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
// 超音波センサーの値を格納する場所
float dist = 0.00;

int point = 0;

//=====================================================================================//
void BMX055_Init() {
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x0F); // Select PMU_Range register
  Wire.write(0x03); // Range = +/- 2g
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x10); // Select PMU_BW register
  Wire.write(0x08); // Bandwidth = 7.81 Hz
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x11); // Select PMU_LPW register
  Wire.write(0x00); // Normal mode, Sleep duration = 0.5ms
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x0F); // Select Range register
  Wire.write(0x04); // Full scale = +/- 125 degree/s
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x10); // Select Bandwidth register
  Wire.write(0x07); // ODR = 100 Hz
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x11); // Select LPM1 register
  Wire.write(0x00); // Normal mode, Sleep duration = 2ms
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B); // Select Mag register
  Wire.write(0x83); // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B); // Select Mag register
  Wire.write(0x01); // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4C); // Select Mag register
  Wire.write(0x00); // Normal Mode, ODR = 10 Hz
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E); // Select Mag register
  Wire.write(0x84); // X, Y, Z-Axis enabled
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51); // Select Mag register
  Wire.write(0x04); // No. of Repetitions for X-Y Axis = 9
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x52); // Select Mag register
  Wire.write(0x16); // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
}
//=====================================================================================//
void BMX055_Accl() {
  unsigned int data[6];
  for (int i = 0; i < 6; i++) {
    Wire.beginTransmission(Addr_Accl);
    Wire.write((2 + i)); // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Accl, 1); // Request 1 byte of data
    // Read 6 bytes of data
    // xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data to 12-bits
  xAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
  if (xAccl > 2047)
    xAccl -= 4096;
  yAccl = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
  if (yAccl > 2047)
    yAccl -= 4096;
  zAccl = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
  if (zAccl > 2047)
    zAccl -= 4096;
  xAccl = xAccl * 0.0098; // range = +/-2g
  yAccl = yAccl * 0.0098; // range = +/-2g
  zAccl = zAccl * 0.0098; // range = +/-2g

  xCalibAccl = xAccl + c_a_x;
  yCalibAccl = yAccl + c_a_y;
  zCalibAccl = zAccl + c_a_z;
}
//=====================================================================================//
void BMX055_Gyro() {
  unsigned int data[6];
  for (int i = 0; i < 6; i++) {
    Wire.beginTransmission(Addr_Gyro);
    Wire.write((2 + i)); // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Gyro, 1); // Request 1 byte of data
    // Read 6 bytes of data
    // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xGyro = (data[1] * 256) + data[0];
  if (xGyro > 32767)
    xGyro -= 65536;
  yGyro = (data[3] * 256) + data[2];
  if (yGyro > 32767)
    yGyro -= 65536;
  zGyro = (data[5] * 256) + data[4];
  if (zGyro > 32767)
    zGyro -= 65536;

  xGyro = xGyro * 0.0038; //  Full scale = +/- 125 degree/s
  yGyro = yGyro * 0.0038; //  Full scale = +/- 125 degree/s
  zGyro = zGyro * 0.0038; //  Full scale = +/- 125 degree/s
}
//=====================================================================================//
void BMX055_Mag() {
  unsigned int data[8];
  for (int i = 0; i < 8; i++) {
    Wire.beginTransmission(Addr_Mag);
    Wire.write((0x42 + i)); // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Mag, 1); // Request 1 byte of data
    // Read 6 bytes of data
    // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xMag = ((data[1] << 5) | (data[0] >> 3));
  if (xMag > 4095)
    xMag -= 8192;
  yMag = ((data[3] << 5) | (data[2] >> 3));
  if (yMag > 4095)
    yMag -= 8192;
  zMag = ((data[5] << 7) | (data[4] >> 1));
  if (zMag > 16383)
    zMag -= 32768;

  xCalibMag = xMag + c_m_x;
  yCalibMag = yMag + c_m_y;
  zCalibMag = zMag + c_m_z;
}

void check_bmx() {
  int xmax = -360;
  int xmin = 360;
  int ymax = -360;
  int ymin = 360;
  BMX055_Mag();
  if (xMag != 0 && yMag != 0) {
    xmax = xMag;
    xmin = xMag;
    ymax = yMag;
    ymin = yMag;
  }
  while (true) {
    BMX055_Mag();
    if (xMag != 0 && yMag != 0) {
      if (xMag > xmax)
        xmax = xMag;
      if (xMag < xmin)
        xmin = xMag;
      if (yMag > ymax)
        ymax = yMag;
      if (yMag < ymin)
        ymin = yMag;
    }
    Serial.print(xmin);
    Serial.print(",");
    Serial.print(xmax);
    Serial.print(",");
    Serial.print(ymin);
    Serial.print(",");
    Serial.println(ymax);
  }
}

float tan2angle(float x, float y) { return atan2(y, x); }

void get_angle() {
  CompVecSize = std::pow((std::pow(xAccl, 2.0) + std::pow(zAccl, 2.0)), 0.5);
  bodyasin = asin(zAccl / CompVecSize);
  if (xAccl < 0) {
    bodyrad = 1.57 - bodyasin;
  } else {
    bodyrad = bodyasin - 1.57;
  }
  yaw = tan2angle(yCalibMag, xCalibMag);
}

// TCA9548Aのi2cをスイッチする関数
void SWITCH_BUS_TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();
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
  if (lRGB[0] - lRGB[1] < -30 && (lRGB[0] + lRGB[1] + lRGB[2]) / 3 < 90) {
    sensors[6] = 2;
  }
  if (rRGB[0] - rRGB[1] < -30 && (rRGB[0] + rRGB[1] + rRGB[2]) / 3 < 90) {
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

void rotate_NC(float angle, float error) {
  BMX055_Accl();
  BMX055_Gyro();
  BMX055_Mag();
  get_angle();
  float pres_angle = bodyrad;
  float target = yaw + angle;
  float origin = target;
  if (angle > 0) {
    if (target > PI)
      target -= 2 * PI;
  } else if (angle < 0) {
    if (target < -PI)
      target += 2 * PI;
  }
  Serial.println("start rotating");
  if (angle > 0) {
    drive_motor(-100, 100);
    delay(100);
  } else if (angle < 0) {
    drive_motor(100, -100);
    delay(100);
  }
  while (decide(yaw, target, origin, error)) {
    BMX055_Accl();
    BMX055_Gyro();
    BMX055_Mag();
    get_angle();
    if (angle > 0) {
      drive_motor(-60, 60);
    } else if (angle < 0) {
      drive_motor(60, -60);
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

void rotate(float angle, float error) {
  BMX055_Accl();
  BMX055_Gyro();
  BMX055_Mag();
  get_angle();
  float pres_angle = bodyrad;
  float target = yaw + angle;
  float origin = target;
  if (angle > 0) {
    if (target > PI)
      target -= 2 * PI;
  } else if (angle < 0) {
    if (target < -PI)
      target += 2 * PI;
  }
  Serial.println("start rotating");
  if (angle > 0) {
    drive_motor(-100, 100);
    delay(100);
  } else if (angle < 0) {
    drive_motor(100, -100);
    delay(100);
  }
  while (decide(yaw, target, origin, error)) {
    BMX055_Accl();
    BMX055_Gyro();
    BMX055_Mag();
    get_angle();
    if (angle > 0) {
      drive_motor(-80, 80);
    } else if (angle < 0) {
      drive_motor(80, -80);
    }
    get_color();
    calculate_color();
    get_mono();
    setled(3, 3, 0);
  }
  while (sensors[4] == 1) {
    get_color();
    calculate_color();
    get_mono();
    if (angle > 0) {
      drive_motor(-70, 70);
    } else if (angle < 0) {
      drive_motor(70, -70);
    }
    setled(3, 3, 3);
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
    rotate_NC(PI / 2, 0.2);
    while (dist > 15) {
      drive_motor(70, 70);
    }
    rotate_NC(PI / 2, 0.2);
    while (sensors[3] == 1 && sensors[4] == 1 && sensors[5] == 1) {
      get_color();
      calculate_color();
      get_mono();
      p_linetrace();
    }
  }
}

void judge_obstacle() {
  if (dist < 5 && dist != 0) {
    Serial.println("found obstacle");
    int now = millis() + 500;
    while (millis() < now) {
      get_color();
      calculate_color();
      get_mono();
      drive_motor(-70, -70);
    }
    rotate_NC(PI / 2, 0.2);
    while (sensors[3] == 1 && sensors[4] == 1 && sensors[5] == 1) {
      get_mono();
      drive_motor(120, 40);
    }
    rotate_NC(PI / 2, 0.2);
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
      rotate(PI / 2, 0.90);
    }
    if (l == false && r == true) {
      rotate(-PI / 2, 0.90);
    }
    if (l == true && r == true) {
      rotate(PI, 0.90);
    }
    Serial.print(checkline);
    Serial.print(",");
    Serial.print(l);
    Serial.print(",");
    Serial.println(r);
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("This is ESP32, Sending message to PC on Serial Connection.");

  Serial.println("Setting up i2c...");
  Wire.begin();
  Wire.setClock(400000);
  BMX055_Init();
  init_color();
  Serial.println("i2c Ok!");
  init_motor();
}

int now = 0;

void loop() {
  BMX055_Accl();
  BMX055_Gyro();
  BMX055_Mag();
  get_angle();

  get_color();
  calculate_color();
  get_mono();
  // debug_mono();
  //  check_bmx();
  judge_obstacle();
  judge_gap();
  judge_green();
  if (bodyrad > 0.1) {
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
  // Serial.println(dist);
  // debug_mono();
  /*Serial.print(lHSV[0]);
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
  Serial.println(rColor[3]);*/
  Serial.println(sensors[8]);
}