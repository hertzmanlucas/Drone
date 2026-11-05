const int PIN_X = 35;   // på ESP32: bruk ADC1-pinner (32–39) hvis WiFi er på
const int PIN_Y = 34;
const int PIN_SW = 25;

int midtX, midtY;
const float DODSONE = 0.08;

float lesAkse(int pin, int midt) {
  float v = (analogRead(pin) - midt) / 2048.0;   // 512.0 på Arduino Uno
  if (fabs(v) < DODSONE) return 0;
  return constrain(v, -1.0, 1.0);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_SW, INPUT_PULLUP);
  delay(100);
  midtX = analogRead(PIN_X);   // kalibrer: ikke rør spaken ved oppstart
  midtY = analogRead(PIN_Y);
}

void loop() {
  float x = lesAkse(PIN_X, midtX);
  float y = lesAkse(PIN_Y, midtY);
  bool trykket = digitalRead(PIN_SW) == LOW;

  float vinkel = atan2(y, x) * 180.0 / PI;   // -180..180 grader
  float utslag = sqrt(x*x + y*y);

  Serial.printf("x=%.2f y=%.2f vinkel=%.0f utslag=%.2f knapp=%d\n",
                x, y, vinkel, utslag, trykket);
  delay(50);
}