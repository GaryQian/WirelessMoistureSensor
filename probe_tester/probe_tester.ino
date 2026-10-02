// Wireless soil moisture bridge — PROBE TESTER (build phase 1)
//
// Board: Seeed XIAO ESP32-C6, powered from a laptop over USB.
// Tools > USB CDC On Boot: Enabled. Serial Monitor at 115200, "Newline".
//
// Measures the watering controller's DC resistive-probe input:
//   - open-circuit probe voltage, and whether it's steady or pulsed
//   - the controller's internal (source) resistance, from one known resistor
//   - the resistance at which it flips wet <-> dry, as you turn the 100k pot
//   - a suggested wet resistor (about half the lowest trip resistance)
//
// Front end (divider, high impedance):
//
//   Probe + --[ R1 470k ]--+-- A0
//                          |
//               GND --[ R2 100k ]
//
//   Probe - ----------------- GND
//
// Reads up to about 17 V. Find which terminal is + with the multimeter first;
// the sketch warns if the leads are reversed (that polarity reads as 0 V and
// is harmless through 470k). The tester loads the probe with about 570k,
// which the maths below accounts for; above ~300k trip resistance that load
// dominates and the numbers get rough, so trust the multimeter cross-check.
//
// Baseline ohmmeter readings of the old resistive probe:
//   out of soil        ~4 M      totally dry soil    ~1 M
//   slightly damp      2-10 k    thoroughly wet      500-1500
// So the trip point is probably somewhere from ~10k to ~1M. Use a 1M pot, or
// the 100k pot in series with fixed 100k/470k resistors to reach higher.
//
// Procedure (type the letter, then Enter):
//   o  Leads on the probe terminals, nothing else across them -> open circuit.
//      Also note what the controller shows: DRY, or a sensor fault? The
//      receiver's relay looks like this when it signals dry.
//   k  Add a known resistor across the terminals (47k is a good start; nearer
//      the trip point is more accurate) -> source resistance.
//   t  Swap the resistor for the pot, start at 0 ohms (wet). Turn it slowly
//      until the controller says DRY and press d. Turn it back until it
//      says WET and press w. Repeat a few times; q to leave.
//   r  Report and suggested wet resistor.
// Cross-check: at a flip, unclip the pot and measure it with the multimeter;
// it should be close to the Rpot figure.

// ---- Settings --------------------------------------------------------------

#define PROBE_ADC_PIN A0

// Divider resistors (use the values you fitted). For probes under ~5 V,
// R1 = 100k gives better resolution (range about 6 V).
#define R1_OHMS 470000.0f   // probe + to A0
#define R2_OHMS 100000.0f   // A0 to GND

// Capture window per measurement. Some controllers pulse the probe only
// occasionally; if a reading says "none" but the multimeter shows voltage,
// make this longer.
#define WINDOW_MS 1000

#define MAX_SAMPLES 8000     // spread evenly over WINDOW_MS (8 kHz at 1 s)
#define NO_SIGNAL_MV 50.0f   // below this the input counts as 0 V
#define MAX_TRIPS 10

// Fake-probe values matched to the ohmmeter baseline above.
#define DEFAULT_WET_OHMS 1000.0f      // thoroughly wet soil: 500-1500
#define DEFAULT_DRY_OHMS 1000000.0f   // totally dry soil: ~1M
// Keep the wet resistor at least this many times below the lowest trip point.
#define WET_MARGIN 4.0f

// ---- Front-end maths -------------------------------------------------------

// Input volts per volt at A0.
static const float GAIN = (R1_OHMS + R2_OHMS) / R2_OHMS;
// Resistance the tester adds across the probe terminals.
static const float TESTER_R = R1_OHMS + R2_OHMS;

struct Capture {
  int n;
  float rateHz;
  float minMv, maxMv, meanMv;
  float levelMv;     // average while "on"; used for the resistance maths
  const char *kind;  // "none", "steady", "pulsed"
  float pulseHz;     // pulses per second; 0 if steady
  float duty;        // fraction of time "on"
};

static float samplesMv[MAX_SAMPLES];

// Measurements.
static bool haveOpen = false;
static Capture openCap;
static float sourceR = -1;  // Thevenin resistance with the tester attached
static float tripDryMv[MAX_TRIPS], tripWetMv[MAX_TRIPS];
static int tripDryCount = 0, tripWetCount = 0;

static Capture capture(uint32_t windowMs) {
  Capture c = {};
  // Spread the samples evenly over the whole window.
  uint32_t intervalUs = windowMs * 1000UL / MAX_SAMPLES;
  uint32_t start = micros();
  uint32_t next = start;
  while (micros() - start < windowMs * 1000UL && c.n < MAX_SAMPLES) {
    while ((int32_t)(micros() - next) < 0) {}
    next += intervalUs;
    samplesMv[c.n++] = analogReadMilliVolts(PROBE_ADC_PIN) * GAIN;
  }
  float seconds = (micros() - start) / 1e6f;
  c.rateHz = c.n / seconds;

  double sum = 0;
  c.minMv = 1e9f;
  c.maxMv = 0;
  for (int i = 0; i < c.n; i++) {
    float v = samplesMv[i];
    sum += v;
    if (v < c.minMv) c.minMv = v;
    if (v > c.maxMv) c.maxMv = v;
  }
  c.meanMv = sum / c.n;

  if (c.maxMv < NO_SIGNAL_MV) {
    c.kind = "none";
    return c;
  }

  // Level = average while "on", so a pulsed probe reads its pulse height.
  float threshold = 0.5f * c.maxMv;
  double onSum = 0;
  int onCount = 0, edges = 0;
  bool wasOn = false;
  for (int i = 0; i < c.n; i++) {
    bool on = samplesMv[i] > threshold;
    if (on) {
      onSum += samplesMv[i];
      onCount++;
    }
    if (on && !wasOn && i > 0) edges++;
    wasOn = on;
  }
  c.levelMv = onSum / onCount;
  c.duty = (float)onCount / c.n;
  if (c.duty > 0.95f) {
    c.kind = "steady";
  } else {
    c.kind = "pulsed";
    c.pulseHz = edges / seconds;
  }
  return c;
}

static void printCapture(const Capture &c) {
  Serial.printf("  %s: level %.2f V | min %.2f max %.2f mean %.2f V",
                c.kind, c.levelMv / 1000, c.minMv / 1000, c.maxMv / 1000, c.meanMv / 1000);
  if (strcmp(c.kind, "pulsed") == 0) Serial.printf(" | %.1f pulses/s, %.0f%% on", c.pulseHz, c.duty * 100);
  Serial.printf(" | %d samples @ %.0f/s\n", c.n, c.rateHz);
}

static String formatOhms(float r) {
  if (r < 0) return "?";
  if (isinf(r)) return "open";
  char buf[24];
  if (r >= 1e6f) snprintf(buf, sizeof(buf), "%.2f M", r / 1e6f);
  else if (r >= 1e3f) snprintf(buf, sizeof(buf), "%.2f k", r / 1e3f);
  else snprintf(buf, sizeof(buf), "%.0f ", r);
  return String(buf) + "ohm";
}

static float parallel(float a, float b) {
  if (isinf(a)) return b;
  return a * b / (a + b);
}

// Resistance across the terminals (the pot) that gives this level, using the
// open-circuit level and source resistance. Both were measured with the tester
// attached, so its loading cancels out.
static float potFromLevel(float levelMv) {
  if (!haveOpen || sourceR < 0) return -1;
  float voc = openCap.levelMv;
  if (levelMv <= 0) return 0;
  if (levelMv >= voc) return INFINITY;
  return sourceR * levelMv / (voc - levelMv);
}

// What the controller actually saw: the pot in parallel with the tester.
// Use this when choosing the wet resistor (the tester won't be there).
static float effectiveFromLevel(float levelMv) {
  float pot = potFromLevel(levelMv);
  return pot < 0 ? -1 : parallel(pot, TESTER_R);
}

static float nearestE12Below(float r) {
  static const float E12[] = {10, 12, 15, 18, 22, 27, 33, 39, 47, 56, 68, 82};
  float decade = powf(10, floorf(log10f(r)) - 1);
  float best = 10 * decade;
  for (float e : E12) {
    if (e * decade <= r) best = e * decade;
  }
  return best;
}

static void drainInput() {
  delay(20);
  while (Serial.available()) Serial.read();
}

// Reads a value like "10000", "10k", "4.7k" or "1M". Returns <= 0 on failure.
static float readOhms() {
  drainInput();
  Serial.setTimeout(120000);
  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return -1;
  float mult = 1;
  char last = s.charAt(s.length() - 1);
  if (last == 'k' || last == 'K') mult = 1e3f;
  if (last == 'm' || last == 'M') mult = 1e6f;
  if (mult != 1) s.remove(s.length() - 1);
  return s.toFloat() * mult;
}

static void cmdOpen() {
  Serial.println("Open circuit: leads on the probe terminals, nothing else across them.");
  Capture c = capture(WINDOW_MS);
  printCapture(c);
  if (strcmp(c.kind, "none") == 0) {
    Serial.println("  No voltage seen. Leads reversed? Swap them. If the multimeter shows");
    Serial.println("  voltage, the probe may pulse rarely: lengthen WINDOW_MS.");
    return;
  }
  if (c.levelMv > 3000 * GAIN) {
    Serial.println("  WARNING: at the top of the range; use a larger R1 for accurate readings.");
  }
  openCap = c;
  haveOpen = true;
  sourceR = -1;  // needs re-measuring against the new open-circuit level
  Serial.printf("  Saved: open-circuit %.2f V (%s). Next: k with a known resistor.\n",
                c.levelMv / 1000, c.kind);
}

static void cmdKnown() {
  if (!haveOpen) {
    Serial.println("Run o (open circuit) first.");
    return;
  }
  Serial.println("Clip a known resistor across the probe terminals (47k is a good start),");
  Serial.println("then type its value measured with the multimeter (e.g. 9870, 10k, 4.7k):");
  float rk = readOhms();
  if (rk <= 0) {
    Serial.println("  Invalid value.");
    return;
  }
  Capture c = capture(WINDOW_MS);
  printCapture(c);
  if (c.levelMv <= 0 || c.levelMv >= openCap.levelMv) {
    Serial.println("  Level not between 0 and open circuit; try a resistor closer to the trip point.");
    return;
  }
  // Tester stays attached for every reading, so solve in its frame: the known
  // resistor against the (source || tester) Thevenin equivalent.
  sourceR = rk * (openCap.levelMv - c.levelMv) / c.levelMv;
  Serial.printf("  %s gives %.2f V -> source resistance %s\n",
                formatOhms(rk).c_str(), c.levelMv / 1000, formatOhms(sourceR).c_str());
  float ratio = c.levelMv / openCap.levelMv;
  if (ratio < 0.1f || ratio > 0.9f) {
    Serial.println("  Level is near an end of the range; a resistor nearer the source resistance is more accurate.");
  }
}

static void cmdTrip() {
  if (sourceR < 0) Serial.println("  (run o and k first to see resistance live; voltages are still recorded)");
  Serial.println("Live: pot across the terminals, start at 0 ohm (wet). Turn slowly.");
  Serial.println("  d = controller just flipped to DRY, w = just flipped to WET, q = done");
  drainInput();
  while (true) {
    Capture last = capture(WINDOW_MS);
    float pot = potFromLevel(last.levelMv);
    Serial.printf("  %.3f V  Rpot %s  (%s)\n", last.levelMv / 1000,
                  formatOhms(pot).c_str(), last.kind);

    while (Serial.available()) {
      char c = Serial.read();
      if (c == 'q') {
        Serial.printf("Recorded %d dry and %d wet flips. r for the report.\n", tripDryCount, tripWetCount);
        return;
      }
      if (c == 'd' && tripDryCount < MAX_TRIPS) {
        tripDryMv[tripDryCount++] = last.levelMv;
        Serial.printf("  >> DRY flip #%d at %.3f V, Rpot %s\n", tripDryCount,
                      last.levelMv / 1000, formatOhms(pot).c_str());
      }
      if (c == 'w' && tripWetCount < MAX_TRIPS) {
        tripWetMv[tripWetCount++] = last.levelMv;
        Serial.printf("  >> WET flip #%d at %.3f V, Rpot %s\n", tripWetCount,
                      last.levelMv / 1000, formatOhms(pot).c_str());
      }
    }
  }
}

static void printTrips(const char *label, const float *mv, int count, float &lowestEff) {
  for (int i = 0; i < count; i++) {
    float pot = potFromLevel(mv[i]);
    float eff = effectiveFromLevel(mv[i]);
    Serial.printf("  %s #%d: %.3f V  Rpot %s  effective %s\n", label, i + 1, mv[i] / 1000,
                  formatOhms(pot).c_str(), formatOhms(eff).c_str());
    if (eff > 0 && !isinf(eff) && eff < lowestEff) lowestEff = eff;
  }
}

static void cmdReport() {
  Serial.println();
  Serial.println("========== PROBE REPORT ==========");
  Serial.printf("Front end: gain %.2f, tester load %s\n", GAIN, formatOhms(TESTER_R).c_str());

  if (!haveOpen) {
    Serial.println("Open circuit: not measured (o)");
  } else {
    Serial.println("Open circuit:");
    printCapture(openCap);
  }
  Serial.printf("Source resistance (with tester): %s%s\n", formatOhms(sourceR).c_str(),
                sourceR < 0 ? " (not measured: k)" : "");

  float lowestEff = INFINITY;
  if (tripDryCount + tripWetCount == 0) {
    Serial.println("Trip points: none recorded (t)");
  } else {
    Serial.println("Trip points:");
    printTrips("to DRY", tripDryMv, tripDryCount, lowestEff);
    printTrips("to WET", tripWetMv, tripWetCount, lowestEff);
  }

  if (!isinf(lowestEff)) {
    Serial.printf("Lowest trip (effective): %s\n", formatOhms(lowestEff).c_str());
    if (lowestEff >= DEFAULT_WET_OHMS * WET_MARGIN) {
      Serial.printf("SUGGESTED WET RESISTOR: %s (matches thoroughly wet soil; %.0fx below the trip)\n",
                    formatOhms(DEFAULT_WET_OHMS).c_str(), lowestEff / DEFAULT_WET_OHMS);
    } else {
      float wet = nearestE12Below(lowestEff / WET_MARGIN);
      Serial.printf("SUGGESTED WET RESISTOR: %s (trip is low, so below the usual %s)\n",
                    formatOhms(wet).c_str(), formatOhms(DEFAULT_WET_OHMS).c_str());
    }
    if (DEFAULT_DRY_OHMS > lowestEff * WET_MARGIN) {
      Serial.printf("Optional DRY RESISTOR on the relay's NO contact: %s (matches dry soil)\n",
                    formatOhms(DEFAULT_DRY_OHMS).c_str());
    } else {
      Serial.println("Trip is close to dry-soil resistance; signal dry with the relay open (no dry resistor).");
    }
  } else if (tripDryCount + tripWetCount > 0) {
    Serial.println("Can't compute resistances yet: run o and k, then r again.");
  }
  Serial.println("Still to check by eye: does the controller water the whole time the");
  Serial.println("probe reads dry, or run its own fixed cycle?");
  Serial.println("==================================");
}

static void printHelp() {
  Serial.println("o=open circuit  k=known resistor  t=trip (live)  r=report  h=help");
}

void setup() {
  Serial.begin(115200);
  unsigned long waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) delay(10);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(PROBE_ADC_PIN, INPUT);

  Serial.println();
  Serial.println("=== Controller probe tester (DC) ===");
  Serial.printf("Input range 0 to about %.1f V, tester load %s\n",
                3100 * GAIN / 1000, formatOhms(TESTER_R).c_str());
  printHelp();
}

void loop() {
  if (!Serial.available()) {
    delay(10);
    return;
  }
  char c = Serial.read();
  switch (c) {
    case 'o': cmdOpen(); break;
    case 'k': cmdKnown(); break;
    case 't': cmdTrip(); break;
    case 'r': cmdReport(); break;
    case 'h': case '?': printHelp(); break;
    default: return;  // ignore newlines etc.
  }
  printHelp();
}
