// Joystick over radio, SENDER: leser joysticken og sender x, y og knapp
// over nRF24L01. Last opp joystickReceiver på den andre ESP32-en for å se
// verdiene der.
//
// Slår sammen Test_joystick og radioSender.
//
// Trenger biblioteket "RF24" av TMRh20 (Library Manager).
//
// Kobling nRF24L01 -> ESP32 (lik på begge):
//  - VCC  -> 3.3V (IKKE 5V, da ødelegges modulen)
//  - GND  -> GND
//  - CE   -> GPIO 2
//  - CSN  -> GPIO 4
//  - SCK  -> GPIO 5
//  - MOSI -> GPIO 18
//  - MISO -> GPIO 21
//  - Tips: en kondensator på ca. 10 µF mellom VCC og GND rett ved modulen
//    fjerner de fleste "sender ikke"-problemer (modulen trekker strømtopper)
//
// Kobling joystick -> ESP32:
//  - VCC -> 3.3V, GND -> GND
//  - VRx -> GPIO 35
//  - VRy -> GPIO 34
//  - SW  -> GPIO 25
//
// Ikke rør spaken mens ESP32-en starter, da kalibreres midtstillingen.
// Serial Monitor (115200 baud) viser hva som sendes og om mottakeren svarte.

#include <SPI.h>
#include <RF24.h>

const int CE_PIN = 2;
const int CSN_PIN = 4;
const int SCK_PIN = 5;
const int MOSI_PIN = 18;
const int MISO_PIN = 21;

const int PIN_X = 35;    // ADC1-pinner (32-39), virker også med WiFi på
const int PIN_Y = 34;
const int PIN_SW = 25;

const float DODSONE = 0.08;

// Disse må være like på sender og mottaker
const uint8_t ADDRESS[6] = "DRONE";   // 5 byte adresse (+ avsluttende null)
const uint8_t CHANNEL = 108;          // 2508 MHz, over de fleste WiFi-kanaler

const int SEND_INTERVAL_MS = 50;      // 20 sendinger i sekundet

// Pakken som sendes. Må være HELT lik i joystickReceiver.
// Aksene sendes som heltall -1000..1000 (= -1.0..1.0), det er mindre og
// tryggere over radio enn float.
struct __attribute__((packed)) JoystickPakke {
  int16_t x;
  int16_t y;
  uint8_t knapp;   // 1 = trykket
};

RF24 radio(CE_PIN, CSN_PIN);
int midtX, midtY;
unsigned long lastSend = 0;

float lesAkse(int pin, int midt) {
  float v = (analogRead(pin) - midt) / 2048.0;
  if (fabs(v) < DODSONE) return 0;
  return constrain(v, -1.0, 1.0);
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_SW, INPUT_PULLUP);
  delay(100);
  midtX = analogRead(PIN_X);   // kalibrer: ikke rør spaken ved oppstart
  midtY = analogRead(PIN_Y);

  // nRF-en sitter ikke på ESP32 sine standard SPI-pinner, så vi sier fra selv
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);
  if (!radio.begin(&SPI)) {
    Serial.println("Fant ikke nRF24L01, sjekk kobling og 3.3V");
    while (true) {
      delay(1000);
    }
  }

  radio.setChannel(CHANNEL);
  radio.setDataRate(RF24_250KBPS);      // tregest, men lengst rekkevidde
  radio.setPALevel(RF24_PA_LOW);        // lav effekt holder på bordet
  radio.setPayloadSize(sizeof(JoystickPakke));
  radio.openWritingPipe(ADDRESS);
  radio.stopListening();                // vi er sender

  Serial.printf("Sender klar (midt x=%d y=%d)\n", midtX, midtY);
}

void loop() {
  if (millis() - lastSend < SEND_INTERVAL_MS) {
    return;
  }
  lastSend = millis();

  JoystickPakke pakke;
  pakke.x = (int16_t)(lesAkse(PIN_X, midtX) * 1000);
  pakke.y = (int16_t)(lesAkse(PIN_Y, midtY) * 1000);
  pakke.knapp = digitalRead(PIN_SW) == LOW ? 1 : 0;

  // write() venter på kvittering (auto-ack) fra mottakeren,
  // så true betyr at pakken faktisk kom fram
  bool ok = radio.write(&pakke, sizeof(pakke));

  Serial.printf("Sendt: x=%5d y=%5d knapp=%d  %s\n",
                pakke.x, pakke.y, pakke.knapp,
                ok ? "OK" : "ingen svar fra mottaker");
}
