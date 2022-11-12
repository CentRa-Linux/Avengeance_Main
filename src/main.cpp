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
int RR = 0;
int RG = 0;
int RB = 0;
int RIR = 0;
int LR = 0;
int LG = 0;
int LB = 0;
int LIR = 0;
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
    RR =
        high_byte << 8 |
        low_byte; // 1Byte目のデータを8bit左にシフト、OR演算子で2Byte目のデータを結合して、redに代入

    high_byte = Wire.read(); // high_byteに「緑(上位バイト)」のデータ読み込み
    low_byte = Wire.read(); // high_byteに「緑(下位バイト)」のデータ読み込み
    RG =
        high_byte << 8 |
        low_byte; // 1Byte目のデータを8bit左にシフト、OR演算子で2Byte目のデータを結合して、greenに代入

    high_byte = Wire.read(); // high_byteに「青(上位バイト)」のデータ読み込み
    low_byte = Wire.read(); // high_byteに「青(下位バイト)」のデータ読み込み
    RB =
        high_byte << 8 |
        low_byte; // 1Byte目のデータを8bit左にシフト、OR演算子で2Byte目のデータを結合して、blueに代入

    high_byte = Wire.read(); // high_byteに「赤外(上位バイト)」のデータ読み込み
    low_byte = Wire.read(); // high_byteに「赤外(下位バイト)」のデータ読み込み
    RIR =
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
    LR =
        high_byte << 8 |
        low_byte; // 1Byte目のデータを8bit左にシフト、OR演算子で2Byte目のデータを結合して、redに代入

    high_byte = Wire.read(); // high_byteに「緑(上位バイト)」のデータ読み込み
    low_byte = Wire.read(); // high_byteに「緑(下位バイト)」のデータ読み込み
    LG =
        high_byte << 8 |
        low_byte; // 1Byte目のデータを8bit左にシフト、OR演算子で2Byte目のデータを結合して、greenに代入

    high_byte = Wire.read(); // high_byteに「青(上位バイト)」のデータ読み込み
    low_byte = Wire.read(); // high_byteに「青(下位バイト)」のデータ読み込み
    LB =
        high_byte << 8 |
        low_byte; // 1Byte目のデータを8bit左にシフト、OR演算子で2Byte目のデータを結合して、blueに代入

    high_byte = Wire.read(); // high_byteに「赤外(上位バイト)」のデータ読み込み
    low_byte = Wire.read(); // high_byteに「赤外(下位バイト)」のデータ読み込み
    LIR =
        high_byte << 8 |
        low_byte; // 1Byte目のデータを8bit左にシフト、OR演算子で2Byte目のデータを結合して、IRに代入
  }
  Wire.endTransmission(); // I2Cスレーブ「Arduino Uno」のデータ送信終了
}

void get_mono() {
  mono[0] = analogRead(25);
  mono[1] = analogRead(26);
  mono[2] = analogRead(27);
  mono[3] = analogRead(32);
  mono[4] = analogRead(33);
  mono[5] = analogRead(34);
  mono[6] = analogRead(35);
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