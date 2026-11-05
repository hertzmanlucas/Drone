// Motortest for én motor på BLHeli 4-in-1 ESC, styrt med potmeter via ESP32.
//
// !! TA AV PROPELLEN FØR DU KJØRER DENNE !!
//
// Kobling:
//  - Potmeter: ytterbena til 3.3V og GND (ikke 5V!), midtbenet til POT_PIN
//  - Signal:   MOTOR_PIN -> signalpaden (M1-M4) for motoren på ESC-en
//  - GND:      ESP32 GND -> GND på ESC-en (felles jord, ellers virker ikke signalet)
//  - Batteriet forsyner ESC-en, ESP32 går på USB
//
// Bruk: skru potmeteret helt ned, last opp og koble til batteriet.
// Motoren er låst til potmeteret har stått på null, så den ikke kan starte
// av seg selv hvis potmeteret står oppe når ESP32 starter.
// Serial Monitor (115200 baud) viser potmeter og gass.

const int POT_PIN = 34;     // GPIO 34 er kun input, passer fint til potmeteret
const int MOTOR_PIN = 32;   // må være en pinne som kan sende PWM (ikke 34-39)

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

// ESP32 leser potmeteret som 0-4095. Nederste del er en død sone,
// så støy rundt null ikke får motoren til å rykke.
const int POT_MAX = 4095;
const int POT_DEADBAND = 100;

// Glatting av potmeteret: 0.0 = ingen, nærmere 1.0 = roligere men tregere
const float SMOOTHING = 0.8;
float potFiltered = 0;

// Hvor fort gassen får øke, så motoren ikke rykker hvis potmeteret vris fort
const int UPDATE_INTERVAL_MS = 20;
const int RAMP_STEP_US = 5;        // per oppdatering, gir 250 µs/s

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

void setup() {
  Serial.begin(115200);

  if (!ledcAttach(MOTOR_PIN, PWM_FREQ, PWM_BITS)) {
    Serial.println("Klarte ikke å sette opp PWM på MOTOR_PIN");
    while (true) {
      delay(1000);
    }
  }

  // Armering: ESC-en må se nullgass før den godtar noe annet.
  // Koble til batteriet nå hvis det ikke allerede står i.
  writePulse(MIN_US);
  Serial.println("Armerer ESC (nullgass i 3 s), vent på pipene...");
  delay(3000);

  // Start filteret på faktisk verdi, ellers ser det ut som null de første
  // målingene og motoren blir låst opp selv om potmeteret står oppe
  potFiltered = analogRead(POT_PIN);
}

void loop() {
  if (millis() - lastUpdate < UPDATE_INTERVAL_MS) {
    return;
  }
  lastUpdate = millis();

  potFiltered = SMOOTHING * potFiltered + (1 - SMOOTHING) * analogRead(POT_PIN);
  int targetUs = potToPulse(potFiltered);

  // Sikkerhetslås: ingen gass før potmeteret har vært helt nede
  if (!unlocked) {
    if (targetUs == MIN_US) {
      unlocked = true;
      Serial.println("Potmeteret står på null, motoren er klar");
    } else {
      targetUs = MIN_US;
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
    if (!unlocked) {
      Serial.println("Låst: skru potmeteret helt ned for å starte");
    } else {
      Serial.printf("Pot: %4d  Gass: %4.1f %% (%d µs)\n",
                    (int)potFiltered, (currentUs - MIN_US) / 10.0, currentUs);
    }
  }
}
