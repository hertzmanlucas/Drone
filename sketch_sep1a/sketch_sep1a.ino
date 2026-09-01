#include <Wire.h>

const uint8_t MPU = 0x68;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();

  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true); 
}

void loop() {
  // put your main code here, to run repeatedly:
  // Si fra hvor vi vil begynne å lese
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);          // ACCEL_XOUT_H
  Wire.endTransmission(false);   // false = ikke slipp bussen

  // Hent 6 byte: to per akse
  Wire.requestFrom(MPU, 6, true);

  int16_t raaX = Wire.read() << 8 | Wire.read();
  int16_t raaY = Wire.read() << 8 | Wire.read();
  int16_t raaZ = Wire.read() << 8 | Wire.read();

  // Gjør om til g (standardområdet er ±2g)
  float aX = raaX / 16384.0;
  float aY = raaY / 16384.0;
  float aZ = raaZ / 16384.0;

  Serial.print("X: "); Serial.print(aX, 2);
  Serial.print("  Y: "); Serial.print(aY, 2);
  Serial.print("  Z: "); Serial.println(aZ, 2);

  delay(200);
}
