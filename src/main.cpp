// Avengeance's main program for ESP32

// i2c Adresses
// BMX055: ACCELEROMETER:0x19 GYRO:0x69 MAGNETIC:0x13
// TCA9548A: 0x70
// S11059-02DT: 0x2A

#include <Arduino.h>
#include <Wire.h>

// TCA9548Aのi2cをスイッチする関数
void SWITCH_BUS_TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(9600);
  Serial.println("This is ESP32, Sending message to PC on Serial Connection.");
  Serial.println("hogehogehoge");

  Serial.println("Setting up i2c...");
  Wire.begin();
  Wire.setClock(400000);
  Serial.println("i2c Ok!");
}

void loop() {}
