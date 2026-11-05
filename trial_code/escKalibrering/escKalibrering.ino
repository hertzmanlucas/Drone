// ESC-kalibrering: lærer alle fire motorene på 4-in-1 ESC-en samme
// gassområde (1000-2000 µs), så de starter på samme gass.
//
// Bruk denne hvis motorene starter på ulik gass i motor_test_joystick_radio.
// Hver motor har sin egen ESC-brikke med sitt eget lagrede gassområde, så en
// motor som er kalibrert annerledes starter tidligere eller senere enn resten.
//
// !! TA AV PROPELLENE !!
// Under kalibreringen sendes signal for FULL GASS. ESC-en går da i
// kalibrering i stedet for å kjøre motorene, men BARE hvis den får strøm
// mens full gass allerede står på. Er batteriet koblet til fra før, kan
// motorene gå på full gass. Batteriet skal derfor være frakoblet helt til
// sketchen ber deg koble det til.
//
// Kobling: samme som motor_test_joystick_radio (radioen trengs ikke her).
//  - GPIO 32 -> M1,  GPIO 33 -> M2,  GPIO 25 -> M3,  GPIO 26 -> M4
//  - ESP32 GND -> GND på ESC-en (felles jord)
//
// Fremgangsmåte (Serial Monitor på 115200 baud):
//  1. Koble fra batteriet. Last opp denne og åpne Serial Monitor.
//  2. Skriv k og trykk Enter. Nå sendes full gass på alle fire.
//  3. Koble til batteriet. ESC-en piper, og etter noen sekunder kommer en
//     ny pipesekvens som betyr at full gass er lagret.
//  4. Skriv n og trykk Enter. Nå sendes nullgass, og ESC-en piper igjen
//     for å bekrefte at null er lagret.
//  5. Koble fra batteriet og last opp motor_test_joystick_radio igjen.
//
// Begynner motorene å snurre i steg 3: skriv n med en gang (nullgass) eller
// dra ut batteriet. Da var ESC-en allerede armert før full gass kom.
// Starter motorene fortsatt ulikt etterpå, kan kalibrering være slått av i
// ESC-en. Da må innstillingene sjekkes i BLHeliSuite.

const int ANTALL_MOTORER = 4;
const int MOTOR_PINS[ANTALL_MOTORER] = {32, 33, 25, 26};   // M1, M2, M3, M4

// Vanlig servo-PWM: 50 Hz, pulsbredde 1000 µs = stopp, 2000 µs = fullt gass
const int PWM_FREQ = 50;
const int PWM_BITS = 16;
const int PERIOD_US = 1000000 / PWM_FREQ;   // 20000 µs
const int MIN_US = 1000;
const int MAX_US = 2000;

// VENTER:    ingen pulser ut, ESC-en ser ikke noe signal
// FULL_GASS: full gass ut, venter på at batteriet kobles til
// NULLGASS:  nullgass ut, ferdig. Full gass kan ikke sendes igjen uten reset,
//            for nå er ESC-en armert og ville kjørt motorene.
enum Steg { VENTER, FULL_GASS, NULLGASS };
Steg steg = VENTER;

void writeAll(int us) {
  uint32_t duty = (uint32_t)us * ((1UL << PWM_BITS) - 1) / PERIOD_US;
  for (int i = 0; i < ANTALL_MOTORER; i++) {
    ledcWrite(MOTOR_PINS[i], duty);
  }
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < ANTALL_MOTORER; i++) {
    if (!ledcAttach(MOTOR_PINS[i], PWM_FREQ, PWM_BITS)) {
      Serial.printf("Klarte ikke å sette opp PWM på GPIO %d\n", MOTOR_PINS[i]);
      while (true) {
        delay(1000);
      }
    }
    ledcWrite(MOTOR_PINS[i], 0);   // pinnen holdes lav, ingen pulser ennå
  }

  delay(500);
  Serial.println();
  Serial.println("ESC-kalibrering. PROPELLENE SKAL VÆRE AV.");
  Serial.println("Batteriet skal være FRAKOBLET nå.");
  Serial.println("Skriv k og trykk Enter når batteriet er frakoblet.");
}

void loop() {
  if (!Serial.available()) {
    return;
  }
  char c = tolower(Serial.read());

  // Nullgass går alltid, så du kan stoppe alt med n
  if (c == 'n') {
    writeAll(MIN_US);
    if (steg != NULLGASS) {
      steg = NULLGASS;
      Serial.println("Sender nullgass. ESC-en skal pipe for å bekrefte.");
      Serial.println("Ferdig: koble fra batteriet og last opp motor_test_joystick_radio.");
    }
    return;
  }

  if (c == 'k') {
    if (steg == VENTER) {
      steg = FULL_GASS;
      writeAll(MAX_US);
      Serial.println("Sender FULL GASS på alle fire.");
      Serial.println("Koble til batteriet nå og vent på pipene (noen sekunder).");
      Serial.println("Skriv så n og trykk Enter.");
    } else if (steg == NULLGASS) {
      Serial.println("Full gass sendes ikke igjen, ESC-en er armert nå.");
      Serial.println("Koble fra batteriet og trykk reset for å starte på nytt.");
    }
  }
}
