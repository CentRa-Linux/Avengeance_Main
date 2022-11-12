// Avengeance's main program for ESP32

// i2c Adresses
// BMX055: ACCELEROMETER:0x19 GYRO:0x69 MAGNETIC:0x13
// TCA9548A: 0x70
// S11059-02DT: 0x2A
// TPR-105F: IO25, 26, 27, 32, 33, 34, 35

#include <Arduino.h>
#include <Wire.h>
// BMX055 加速度センサのI2Cアドレス
#define Addr_Accl 0x19 // (JP1,JP2,JP3 = Openの時)
// BMX055 ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69 // (JP1,JP2,JP3 = Openの時)
// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13 // (JP1,JP2,JP3 = Openの時)
// S11059のアドレス、定数
#define S11059_ADDR 0x2A
#define CONTROL_MSB 0x00
#define CONTROL_1_LSB 0x89
#define CONTROL_2_LSB 0x09
#define SENSOR_REGISTER 0x03
#define MONO_0 25
#define MONO_1 26
#define MONO_2 27
#define MONO_3 32
#define MONO_4 33
#define MONO_5 34
#define MONO_6 35
// モータードライバのピン
#define PWMA 17
#define PWMB 16
#define AIN1 4
#define BIN1 2
#define AIN2 0
#define BIN2 15
// ライントレース系の定数

// センサーの値を保存するグローバル変数
float xAccl = 0.00;
float yAccl = 0.00;
float zAccl = 0.00;
float xGyro = 0.00;
float yGyro = 0.00;
float zGyro = 0.00;
//機体の角度を保存するグローバル変数
float CompVecSize = 0.00;
float bodyrad = 0.00;
float bodyasin = 0.00;
int xMag = 0;
int yMag = 0;
int zMag = 0;
int rColor[4];
int rHSV[3];
int lColor[4];
int lHSV[3];
int mono[7];

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
}

void get_angle() {
  CompVecSize = std::pow((std::pow(xAccl, 2.0) + std::pow(zAccl, 2.0)), 0.5);
  bodyasin = asin(zAccl / CompVecSize);
  if (xAccl < 0) {
    bodyrad = 1.57 - bodyasin;
  } else {
    bodyrad = bodyasin - 1.57;
  }
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

void calculate_color() {}

void get_mono() {
  mono[0] = analogRead(MONO_0);
  mono[1] = analogRead(MONO_1);
  mono[2] = analogRead(MONO_2);
  mono[3] = analogRead(MONO_3);
  mono[4] = analogRead(MONO_4);
  mono[5] = analogRead(MONO_5);
  mono[6] = analogRead(MONO_6);
}

void init_motor() {
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN2, OUTPUT);
}

void drive_motor(int a, int b) {
  if (a == 0) {
    // BRAKE
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
    ledcWrite(PWMA, a);
  }
  if (a > 0) {
    // CW
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(PWMA, a);
  }
  if (a < 0) {
    // CCW
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(PWMA, a);
  }
  if (b == 0) {
    // BRAKE
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
    ledcWrite(PWMB, b);
  }
  if (b > 0) {
    // CW
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(PWMB, b);
  }
  if (b < 0) {
    // CCW
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(PWMB, b);
  }
}

void simple_linetrace() {
  if (mono[2] < 100 && mono[4] > 100) {
    drive_motor(80, 20);
  }
  if (mono[2] > 100 && mono[4] < 100) {
    drive_motor(20, 80);
  }
  if (mono[2] < 100 && mono[4] < 100) {
    drive_motor(80, 80);
  }
  if (mono[2] > 100 && mono[4] > 100) {
    drive_motor(80, 80);
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
}

void loop() {
  BMX055_Accl();
  BMX055_Gyro();
  BMX055_Mag();
  get_angle();

  get_color();
  get_mono();
  Serial.println(analogRead(27));
}