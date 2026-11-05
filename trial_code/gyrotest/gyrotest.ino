#include <Wire.h>

const uint8_t MPU = 0x68;

// Nullpunkt (bias) for gyroen, målt ved oppstart
float biasX = 0, biasY = 0, biasZ = 0;

// Glatting: 0.0 = ingen, nærmere 1.0 = tregere men roligere plott
const float SMOOTHING = 0.7;
float filtX = 0, filtY = 0, filtZ = 0;

// Leser de tre gyro-aksene som rå 16-bits verdier.
// Returnerer false hvis I2C-lesingen feilet, så vi slipper søppelverdier.
bool readGyroRaw(int16_t &x, int16_t &y, int16_t &z) {
  // Si fra hvor vi vil begynne å lese
  Wire.beginTransmission(MPU);
  Wire.write(0x43);              // GYRO_XOUT_H
  if (Wire.endTransmission(false) != 0) {   // false = ikke slipp bussen
    return false;
  }

  // Hent 6 byte: to per akse
  if (Wire.requestFrom(MPU, (uint8_t)6, (uint8_t)true) != 6) {
    return false;
  }

  // Les én byte om gangen: rekkefølgen på Wire.read() inne i ett uttrykk
  // er ikke garantert av C++, så høy/lav byte kan bli byttet om
  uint8_t xh = Wire.read(), xl = Wire.read();
  uint8_t yh = Wire.read(), yl = Wire.read();
  uint8_t zh = Wire.read(), zl = Wire.read();

  x = (int16_t)((xh << 8) | xl);
  y = (int16_t)((yh << 8) | yl);
  z = (int16_t)((zh << 8) | zl);
  return true;
}

void setup() {
  // Høy baudrate så plottet oppdaterer seg raskt nok
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);         // 400 kHz I2C

  // Vekk MPU-en fra sleep
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);              // PWR_MGMT_1
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Sjekk at vi faktisk snakker med sensoren
  Wire.beginTransmission(MPU);
  Wire.write(0x75);              // WHO_AM_I
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, (uint8_t)1, (uint8_t)true);
  uint8_t who = Wire.read();
  if (who != 0x68) {
    Serial.print("Fant ikke MPU6050, WHO_AM_I = 0x");
    Serial.println(who, HEX);
  }

  // Lavpassfilter i sensoren: 42 Hz båndbredde i stedet for 256 Hz.
  // Dette fjerner det meste av høyfrekvent støy før vi i det hele tatt leser.
  Wire.beginTransmission(MPU);
  Wire.write(0x1A);              // CONFIG
  Wire.write(0x03);              // DLPF_CFG = 3
  Wire.endTransmission(true);

  // Sett gyro-området til ±250 °/s (mest følsomt)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);              // GYRO_CONFIG
  Wire.write(0x00);              // 0x00 = ±250 °/s
  Wire.endTransmission(true);

  delay(100);                    // la filteret sette seg før vi kalibrerer

  // Kalibrering: ligg helt stille i ~1 sekund og finn nullpunktet
  const int samples = 200;
  long sumX = 0, sumY = 0, sumZ = 0;
  int16_t minX = 32767, maxX = -32768;
  int ok = 0;
  for (int i = 0; i < samples; i++) {
    int16_t rawX, rawY, rawZ;
    if (readGyroRaw(rawX, rawY, rawZ)) {
      sumX += rawX;
      sumY += rawY;
      sumZ += rawZ;
      if (rawX < minX) minX = rawX;
      if (rawX > maxX) maxX = rawX;
      ok++;
    }
    delay(5);
  }
  if (ok > 0) {
    biasX = sumX / (float)ok;
    biasY = sumY / (float)ok;
    biasZ = sumZ / (float)ok;
  }

  // Stor spredning betyr at sensoren ble flyttet under kalibreringen
  if (maxX - minX > 200) {
    Serial.println("Sensoren beveget seg under kalibrering - trykk reset");
  }
}

void loop() {
  int16_t rawX, rawY, rawZ;
  if (!readGyroRaw(rawX, rawY, rawZ)) {
    return;                      // hopp over feilet lesing, behold forrige verdi
  }

  // Gjør om til grader per sekund (131 LSB per °/s ved ±250 °/s)
  float gX = (rawX - biasX) / 131.0;
  float gY = (rawY - biasY) / 131.0;
  float gZ = (rawZ - biasZ) / 131.0;

  // Enkelt lavpassfilter i programvare oppå sensorens eget
  filtX = SMOOTHING * filtX + (1 - SMOOTHING) * gX;
  filtY = SMOOTHING * filtY + (1 - SMOOTHING) * gY;
  filtZ = SMOOTHING * filtZ + (1 - SMOOTHING) * gZ;

  // Format for Serial Plotter: "navn:verdi" adskilt med tab gir tre kurver
  Serial.print("X:"); Serial.print(filtX, 2);
  Serial.print("\tY:"); Serial.print(filtY, 2);
  Serial.print("\tZ:"); Serial.println(filtZ, 2);

  delay(200);   // ca. 50 målinger i sekundet
}
