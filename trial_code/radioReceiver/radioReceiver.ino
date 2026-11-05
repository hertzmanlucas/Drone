// Radiotest, MOTTAKER: tar imot potmeterverdien fra radioSender over
// nRF24L01 og bruker den til å styre én motor på BLHeli 4-in-1 ESC.
//
// !! TA AV PROPELLEN FØR DU KJØRER DENNE !!
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
// Kobling motor:
//  - Signal: MOTOR_PIN (GPIO 32) -> signalpaden (M1-M4) for motoren på ESC-en
//  - GND:    ESP32 GND -> GND på ESC-en (felles jord, ellers virker ikke signalet)
//  - Batteriet forsyner ESC-en, ESP32 går på USB
//
// Bruk: skru potmeteret på senderen helt ned, last opp og koble til batteriet.
// Motoren er låst til potmeteret har stått på null, så den ikke kan starte
// av seg selv. Mister vi signalet fra senderen stopper motoren med en gang
// og låses igjen, så den heller ikke kan starte når signalet kommer tilbake.
// Serial Monitor (115200 baud) viser potmeter og gass.

#include <SPI.h>
#include <RF24.h>

const int CE_PIN = 2;
const int CSN_PIN = 4;
const int SCK_PIN = 5;
const int MOSI_PIN = 18;
const int MISO_PIN = 21;

const int MOTOR_PIN = 32;   // må være en pinne som kan sende PWM (ikke 34-39)

// Disse må være like på sender og mottaker
const uint8_t ADDRESS[6] = "DRONE";   // 5 byte adresse (+ avsluttende null)
const uint8_t CHANNEL = 108;          // 2508 MHz, over de fleste WiFi-kanaler

// Stopp motoren hvis det går så lang tid uten noe fra senderen.
// Senderen sender hvert 50. ms, så dette er ca. 5 tapte pakker på rad.
const int SIGNAL_TIMEOUT_MS = 250;

// Vanlig servo-PWM: 50 Hz, pulsbredde 1000 µs = stopp, 2000 µs = fullt gass.
// BLHeli oppdager denne signaltypen automatisk.
const int PWM_FREQ = 50;
const int PWM_BITS = 16;
const int PERIOD_US = 1000000 / PWM_FREQ;   // 20000 µs
const int MIN_US = 1000;
const int MAX_US = 2000;

// Sikkerhetsgrense mens vi tester: potmeteret helt opp gir bare så mye gass
const int MAX_PERCENT = 100;
const int MAX_TEST_US = MIN_US + (MAX_US - MIN_US) * MAX_PERCENT / 100;

// Senderen leser potmeteret som 0-4095. Nederste del er en død sone,
// så støy rundt null ikke får motoren til å rykke.
const int POT_MAX = 4095;
const int POT_DEADBAND = 100;

// Glatting av potmeteret: 0.0 = ingen, nærmere 1.0 = roligere men tregere
const float SMOOTHING = 0.8;
float potFiltered = 0;

// Hvor fort gassen får øke, så motoren ikke rykker hvis potmeteret vris fort
const int UPDATE_INTERVAL_MS = 20;
const int RAMP_STEP_US = 50;        // per oppdatering, gir 250 µs/s

RF24 radio(CE_PIN, CSN_PIN);
uint16_t lastPot = 0;              // siste verdi fra senderen
bool haveSignal = false;
unsigned long lastReceived = 0;

int currentUs = MIN_US;
bool unlocked = false;
unsigned long lastUpdate = 0;
unsigned long lastPrint = 0;

// Gjør om pulsbredde i µs til duty cycle for LEDC
void writePulse(int us) {
  us = constrain(us, MIN_US, MAX_TEST_US);   // siste sperre mot for mye gass
  uint32_t duty = (uint32_t)us * ((1UL << PWM_BITS) - 1) / PERIOD_US;
  ledcWrite(MOTOR_PIN, duty);
}

// Gjør om potmeterverdi til pulsbredde mellom MIN_US og MAX_TEST_US
int potToPulse(float pot) {
  if (pot <= POT_DEADBAND) {
    return MIN_US;
  }
  return MIN_US + (pot - POT_DEADBAND) * (MAX_TEST_US - MIN_US) / (POT_MAX - POT_DEADBAND);
}

// Stopp motoren med en gang og krev at potmeteret går til null igjen
void stopMotor() {
  unlocked = false;
  currentUs = MIN_US;
  writePulse(MIN_US);
}

void setup() {
  Serial.begin(115200);

  // Nullgass ut på motorpinnen først, så ESC-en aldri ser noe annet
  if (!ledcAttach(MOTOR_PIN, PWM_FREQ, PWM_BITS)) {
    Serial.println("Klarte ikke å sette opp PWM på MOTOR_PIN");
    while (true) {
      delay(1000);
    }
  }
  writePulse(MIN_US);

  // nRF-en sitter ikke på ESP32 sine standard SPI-pinner, så vi sier fra selv
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);
  if (!radio.begin(&SPI)) {
    Serial.println("Fant ikke nRF24L01, sjekk kobling og 3.3V");
    while (true) {
      delay(1000);               // motoren står fortsatt på nullgass
    }
  }

  radio.setChannel(CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(sizeof(uint16_t));
  radio.openReadingPipe(1, ADDRESS);
  radio.startListening();               // vi er mottaker

  // Armering: ESC-en må se nullgass før den godtar noe annet.
  // Koble til batteriet nå hvis det ikke allerede står i.
  Serial.println("Armerer ESC (nullgass i 3 s), vent på pipene...");
  delay(3000);
  Serial.println("Mottaker klar, venter på data...");
}

void loop() {
  // Hent alt som har kommet inn, vi bruker bare den nyeste verdien
  while (radio.available()) {
    radio.read(&lastPot, sizeof(lastPot));
    lastReceived = millis();

    if (!haveSignal) {
      haveSignal = true;
      // Start filteret på faktisk verdi, ellers ser det ut som null de første
      // målingene og motoren blir låst opp selv om potmeteret står oppe
      potFiltered = lastPot;
      Serial.println("Har signal fra senderen");
    }
  }

  // Failsafe: ingen pakker på en stund betyr stopp, uansett hva gassen var
  if (haveSignal && millis() - lastReceived > SIGNAL_TIMEOUT_MS) {
    haveSignal = false;
    stopMotor();
    Serial.println("Mistet signalet, motoren er stoppet");
  }

  if (millis() - lastUpdate < UPDATE_INTERVAL_MS) {
    return;
  }
  lastUpdate = millis();

  int targetUs = MIN_US;
  if (haveSignal) {
    potFiltered = SMOOTHING * potFiltered + (1 - SMOOTHING) * lastPot;
    targetUs = potToPulse(potFiltered);

    // Sikkerhetslås: ingen gass før potmeteret har vært helt nede
    if (!unlocked) {
      if (targetUs == MIN_US) {
        unlocked = true;
        Serial.println("Potmeteret står på null, motoren er klar");
      } else {
        targetUs = MIN_US;
      }
    }
  }

  // Øk gassen gradvis, men gå ned med en gang (stopp skal være umiddelbart)
  if (currentUs < targetUs) {
    currentUs = min(currentUs + RAMP_STEP_US, targetUs);
  } else {
    currentUs = targetUs;
  }
  writePulse(currentUs);

  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    if (!haveSignal) {
      Serial.println("Ingen signal fra senderen, motoren står stille");
    } else if (!unlocked) {
      Serial.println("Låst: skru potmeteret helt ned for å starte");
    } else {
      Serial.printf("Pot: %4d  Gass: %4.1f %% (%d µs)\n",
                    (int)potFiltered, (currentUs - MIN_US) / 10.0, currentUs);
    }
  }
}
