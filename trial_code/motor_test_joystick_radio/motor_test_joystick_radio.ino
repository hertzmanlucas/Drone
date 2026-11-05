// Motortest med joystick over radio, MOTTAKER (sitter på dronen).
// Tar imot x, y og knapp fra joystickSender over nRF24L01 og styrer de fire
// motorene på BLHeli 4-in-1 ESC-en.
//
// !! TA AV PROPELLENE FØR DU KJØRER DENNE !!
//
// Sender: last opp joystickSender (uendret) på ESP32-en med joysticken.
// Mottaker: last opp denne på ESP32-en på dronen.
//
// Trenger biblioteket "RF24" av TMRh20 (Library Manager).
//
// Kobling nRF24L01 -> ESP32 (samme som i de andre radiotestene):
//  - VCC  -> 3.3V (IKKE 5V, da ødelegges modulen)
//  - GND  -> GND
//  - CE   -> GPIO 2
//  - CSN  -> GPIO 4
//  - SCK  -> GPIO 5
//  - MOSI -> GPIO 18
//  - MISO -> GPIO 21
//  - Tips: en kondensator på ca. 10 µF mellom VCC og GND rett ved modulen
//
// Kobling motorer -> ESC:
//  - GPIO 32 -> M1,  GPIO 33 -> M2,  GPIO 25 -> M3,  GPIO 26 -> M4
//  - GND:    ESP32 GND -> GND på ESC-en (felles jord, ellers virker ikke signalet)
//  - Batteriet forsyner ESC-en, ESP32 går på USB
//  Har du bare koblet til én motor, sett den på GPIO 32 og bruk M1.
//
// Bruk:
//  - Spaken FRAM (y opp) = gass. Slipp spaken, så går den til midten og
//    motoren stopper. Bakover gir ingen gass.
//  - Trykk ned spaken (knappen) for å bytte motor: M1 -> M2 -> M3 -> M4 -> ALLE.
//    Det går bare mens motorene står stille.
//  - Motorene er låst til spaken har stått i midten, så de ikke kan starte av
//    seg selv. Mister vi signalet stopper alt med en gang og låses igjen.
//  - Starter motoren når du drar spaken BAKOVER? Sett GASS_RETNING til -1.
// Serial Monitor (115200 baud) viser valgt motor, y og gass.

#include <SPI.h>
#include <RF24.h>

const int CE_PIN = 2;
const int CSN_PIN = 4;
const int SCK_PIN = 5;
const int MOSI_PIN = 18;
const int MISO_PIN = 21;

// Må være pinner som kan sende PWM (ikke 34-39), og ikke de som nRF-en bruker
const int ANTALL_MOTORER = 4;
const int MOTOR_PINS[ANTALL_MOTORER] = {32, 33, 25, 26};   // M1, M2, M3, M4

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

// 1 = spaken fram gir gass, -1 = spaken bak gir gass
const int GASS_RETNING = 1;

// Vanlig servo-PWM: 50 Hz, pulsbredde 1000 µs = stopp, 2000 µs = fullt gass.
// BLHeli oppdager denne signaltypen automatisk.
const int PWM_FREQ = 50;
const int PWM_BITS = 16;
const int PERIOD_US = 1000000 / PWM_FREQ;   // 20000 µs
const int MIN_US = 1000;
const int MAX_US = 2000;

// Sikkerhetsgrense mens vi tester: spaken helt fram gir bare så mye gass.
// Øk når dere har sett at alt oppfører seg som det skal.
const int MAX_PERCENT = 50;
const int MAX_TEST_US = MIN_US + (MAX_US - MIN_US) * MAX_PERCENT / 100;

// Hvor fort gassen får øke, så motoren ikke rykker hvis spaken dyttes fort
const int UPDATE_INTERVAL_MS = 20;
const int RAMP_STEP_US = 5;          // per oppdatering, gir 250 µs/s

// Minste tid mellom to knappetrykk, så ett trykk ikke telles to ganger
const int KNAPP_PAUSE_MS = 300;

// Valg av motor: 0-3 = én motor (M1-M4), 4 = alle samtidig
const int ALLE = ANTALL_MOTORER;
int valgtMotor = 0;

RF24 radio(CE_PIN, CSN_PIN);
JoystickPakke siste = {0, 0, 0};
bool haveSignal = false;
unsigned long lastReceived = 0;

bool forrigeKnapp = false;
unsigned long lastKnappTrykk = 0;

int currentUs = MIN_US;
bool unlocked = false;
unsigned long lastUpdate = 0;
unsigned long lastPrint = 0;

// Gjør om pulsbredde i µs til duty cycle for LEDC
void writePulse(int pin, int us) {
  us = constrain(us, MIN_US, MAX_TEST_US);   // siste sperre mot for mye gass
  uint32_t duty = (uint32_t)us * ((1UL << PWM_BITS) - 1) / PERIOD_US;
  ledcWrite(pin, duty);
}

// Valgt motor får gassen, resten står på nullgass
void writeMotors(int us) {
  for (int i = 0; i < ANTALL_MOTORER; i++) {
    bool aktiv = (valgtMotor == ALLE || valgtMotor == i);
    writePulse(MOTOR_PINS[i], aktiv ? us : MIN_US);
  }
}

// Stopp alle motorene med en gang og krev at spaken går til midten igjen
void stopMotors() {
  unlocked = false;
  currentUs = MIN_US;
  writeMotors(MIN_US);
}

void printValgtMotor() {
  if (valgtMotor == ALLE) {
    Serial.println("Valgt: ALLE motorene");
  } else {
    Serial.printf("Valgt: M%d (GPIO %d)\n", valgtMotor + 1, MOTOR_PINS[valgtMotor]);
  }
}

// Gjør om y (-1000..1000) til pulsbredde. Bare én retning gir gass.
int yToPulse(int16_t y) {
  int gass = y * GASS_RETNING;   // 0..1000 når spaken går riktig vei
  if (gass <= 0) {
    return MIN_US;
  }
  gass = min(gass, 1000);
  return MIN_US + (long)gass * (MAX_TEST_US - MIN_US) / 1000;
}

// Bytt motor på knappetrykk (når knappen går fra sluppet til trykket)
void sjekkKnapp() {
  bool trykket = haveSignal && siste.knapp == 1;
  bool nyttTrykk = trykket && !forrigeKnapp;
  forrigeKnapp = trykket;

  if (!nyttTrykk || millis() - lastKnappTrykk < KNAPP_PAUSE_MS) {
    return;
  }
  lastKnappTrykk = millis();

  if (currentUs != MIN_US) {
    Serial.println("Slipp gassen før du bytter motor");
    return;
  }
  valgtMotor = (valgtMotor + 1) % (ANTALL_MOTORER + 1);
  writeMotors(MIN_US);
  printValgtMotor();
}

void setup() {
  Serial.begin(115200);

  // Nullgass ut på alle motorpinnene først, så ESC-en aldri ser noe annet
  for (int i = 0; i < ANTALL_MOTORER; i++) {
    if (!ledcAttach(MOTOR_PINS[i], PWM_FREQ, PWM_BITS)) {
      Serial.printf("Klarte ikke å sette opp PWM på GPIO %d\n", MOTOR_PINS[i]);
      while (true) {
        delay(1000);
      }
    }
    writePulse(MOTOR_PINS[i], MIN_US);
  }

  // nRF-en sitter ikke på ESP32 sine standard SPI-pinner, så vi sier fra selv
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);
  if (!radio.begin(&SPI)) {
    Serial.println("Fant ikke nRF24L01, sjekk kobling og 3.3V");
    while (true) {
      delay(1000);               // motorene står fortsatt på nullgass
    }
  }

  radio.setChannel(CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(sizeof(JoystickPakke));
  radio.openReadingPipe(1, ADDRESS);
  radio.startListening();               // vi er mottaker

  // Armering: ESC-en må se nullgass før den godtar noe annet.
  // Koble til batteriet nå hvis det ikke allerede står i.
  Serial.println("Armerer ESC (nullgass i 3 s), vent på pipene...");
  delay(3000);
  Serial.printf("Mottaker klar, maks gass %d %%. Venter på joysticken...\n", MAX_PERCENT);
  printValgtMotor();
}

void loop() {
  // Hent alt som har kommet inn, vi bruker bare den nyeste pakken
  while (radio.available()) {
    radio.read(&siste, sizeof(siste));
    lastReceived = millis();

    if (!haveSignal) {
      haveSignal = true;
      // Ikke tolk en knapp som allerede er nede som et nytt trykk
      forrigeKnapp = siste.knapp == 1;
      Serial.println("Har signal fra senderen");
    }
  }

  // Failsafe: ingen pakker på en stund betyr stopp, uansett hva gassen var
  if (haveSignal && millis() - lastReceived > SIGNAL_TIMEOUT_MS) {
    haveSignal = false;
    siste = {0, 0, 0};
    stopMotors();
    Serial.println("Mistet signalet, motorene er stoppet");
  }

  if (millis() - lastUpdate < UPDATE_INTERVAL_MS) {
    return;
  }
  lastUpdate = millis();

  sjekkKnapp();

  int targetUs = MIN_US;
  if (haveSignal) {
    targetUs = yToPulse(siste.y);

    // Sikkerhetslås: ingen gass før spaken har vært i midten
    if (!unlocked) {
      if (targetUs == MIN_US) {
        unlocked = true;
        Serial.println("Spaken står i midten, motorene er klare");
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
  writeMotors(currentUs);

  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    if (!haveSignal) {
      Serial.println("Ingen signal fra senderen, motorene står stille");
    } else if (!unlocked) {
      Serial.println("Låst: slipp spaken til midten for å starte");
    } else {
      if (valgtMotor == ALLE) {
        Serial.print("ALLE");
      } else {
        Serial.printf("M%d  ", valgtMotor + 1);
      }
      Serial.printf("  y=%5.2f  Gass: %4.1f %% (%d µs)\n",
                    siste.y / 1000.0, (currentUs - MIN_US) / 10.0, currentUs);
    }
  }
}
