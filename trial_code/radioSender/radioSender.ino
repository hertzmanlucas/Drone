// Radiotest, SENDER: leser potmeteret og sender verdien over nRF24L01.
// Last opp radioReceiver på den andre ESP32-en for å se verdien der.
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
// Potmeter: ytterbena til 3.3V og GND, midtbenet til POT_PIN.
//
// Serial Monitor (115200 baud) viser hva som sendes og om mottakeren svarte.

#include <SPI.h>
#include <RF24.h>

const int CE_PIN = 2;
const int CSN_PIN = 4;
const int SCK_PIN = 5;
const int MOSI_PIN = 18;
const int MISO_PIN = 21;

// GPIO 22 kan ikke lese analoge verdier på ESP32, så potmeteret må stå på
// en ADC-pinne. GPIO 34 er kun input og passer fint (samme som i motorTest).
const int POT_PIN = 34;

// Disse må være like på sender og mottaker
const uint8_t ADDRESS[6] = "DRONE";   // 5 byte adresse (+ avsluttende null)
const uint8_t CHANNEL = 108;          // 2508 MHz, over de fleste WiFi-kanaler

const int SEND_INTERVAL_MS = 50;      // 20 sendinger i sekundet

RF24 radio(CE_PIN, CSN_PIN);
unsigned long lastSend = 0;

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
  radio.setDataRate(RF24_250KBPS);      // tregest, men lengst rekkevidde
  radio.setPALevel(RF24_PA_LOW);        // lav effekt holder på bordet
  radio.setPayloadSize(sizeof(uint16_t));
  radio.openWritingPipe(ADDRESS);
  radio.stopListening();                // vi er sender

  Serial.println("Sender klar");
}

void loop() {
  if (millis() - lastSend < SEND_INTERVAL_MS) {
    return;
  }
  lastSend = millis();

  uint16_t pot = analogRead(POT_PIN);   // 0-4095

  // write() venter på kvittering (auto-ack) fra mottakeren,
  // så true betyr at verdien faktisk kom fram
  bool ok = radio.write(&pot, sizeof(pot));

  Serial.printf("Sendt: %4u  %s\n", pot, ok ? "OK" : "ingen svar fra mottaker");
}
