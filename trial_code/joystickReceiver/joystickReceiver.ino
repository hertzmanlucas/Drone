// Joystick over radio, MOTTAKER: tar imot x, y og knapp fra joystickSender
// over nRF24L01 og skriver dem ut. Styrer ingenting ennå, dette er bare for
// å sjekke at joystick-signalene kommer fram.
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
//
// Serial Monitor (115200 baud) viser joystick-verdiene, eller at signalet
// er borte.

#include <SPI.h>
#include <RF24.h>

const int CE_PIN = 2;
const int CSN_PIN = 4;
const int SCK_PIN = 5;
const int MOSI_PIN = 18;
const int MISO_PIN = 21;

// Disse må være like på sender og mottaker
const uint8_t ADDRESS[6] = "DRONE";   // 5 byte adresse (+ avsluttende null)
const uint8_t CHANNEL = 108;          // 2508 MHz, over de fleste WiFi-kanaler

// Senderen sender hvert 50. ms, så dette er ca. 5 tapte pakker på rad
const int SIGNAL_TIMEOUT_MS = 250;

// Pakken som mottas. Må være HELT lik den i joystickSender.
struct __attribute__((packed)) JoystickPakke {
  int16_t x;       // -1000..1000
  int16_t y;       // -1000..1000
  uint8_t knapp;   // 1 = trykket
};

RF24 radio(CE_PIN, CSN_PIN);
JoystickPakke siste = {0, 0, 0};
bool haveSignal = false;
unsigned long lastReceived = 0;
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);

  // nRF-en sitter ikke på ESP32 sine standard SPI-pinner, så vi sier fra selv
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);
  if (!radio.begin(&SPI)) {
    Serial.println("Fant ikke nRF24L01, sjekk kobling og 3.3V");
    while (true) {
      delay(1000);
    }
  }

  radio.setChannel(CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(sizeof(JoystickPakke));
  radio.openReadingPipe(1, ADDRESS);
  radio.startListening();               // vi er mottaker

  Serial.println("Mottaker klar, venter på data...");
}

void loop() {
  // Hent alt som har kommet inn, vi bruker bare den nyeste pakken
  while (radio.available()) {
    radio.read(&siste, sizeof(siste));
    lastReceived = millis();

    if (!haveSignal) {
      haveSignal = true;
      Serial.println("Har signal fra senderen");
    }
  }

  // Failsafe: ingen pakker på en stund betyr at vi glemmer siste verdi,
  // så en drone senere ikke fortsetter med gammel styring
  if (haveSignal && millis() - lastReceived > SIGNAL_TIMEOUT_MS) {
    haveSignal = false;
    siste = {0, 0, 0};
    Serial.println("Mistet signalet");
  }

  if (millis() - lastPrint < 200) {
    return;
  }
  lastPrint = millis();

  if (!haveSignal) {
    Serial.println("Ingen signal fra senderen");
    return;
  }

  float x = siste.x / 1000.0;   // tilbake til -1.0..1.0
  float y = siste.y / 1000.0;
  float vinkel = atan2(y, x) * 180.0 / PI;   // -180..180 grader
  float utslag = sqrt(x * x + y * y);

  Serial.printf("x=%.2f y=%.2f vinkel=%.0f utslag=%.2f knapp=%d\n",
                x, y, vinkel, utslag, siste.knapp);
}
