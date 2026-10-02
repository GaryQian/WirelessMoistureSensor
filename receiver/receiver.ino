// Wireless soil moisture bridge — RECEIVER
//
// Board: Seeed XIAO ESP32-C6 (Arduino core "esp32" by Espressif, 3.x)
// Tools > USB CDC On Boot: Enabled (for serial output and commands)
//
// Always powered from USB. Receives moisture packets over ESP-NOW, keeps the
// watering schedule, and fakes the controller's old resistive probe with a
// normally-closed relay and a "wet" resistor:
//   relay off (NC closed) -> controller reads WET  (default, and at boot)
//   relay on  (NC open)   -> controller reads DRY  (only while watering)
//
// Fake-probe loop (never connected to the ESP32):
//   probe A -- relay COM
//   relay NC -- 1k   -- probe B   "wet": thoroughly wet soil read 500-1500 ohm
//   relay NO -- 1M   -- probe B   "dry": dry soil read ~1M (optional)
// Leave NO unconnected (open circuit, like the probe out of soil at ~4M) if
// the controller reads that as dry rather than a sensor fault. probe_tester
// suggests values from the measured trip point.
//
// Rules (settings in config.h):
//   - Each packet carries raw ADC readings of A0-A2. SENSORS in config.h says
//     which are fitted and how to convert them; the decision uses the average
//     of the fitted sensors, leaving out any reading outside
//     RAW_VALID_MIN-RAW_VALID_MAX as faulty.
//   - Average < DRY_THRESHOLD_PCT and at least
//     MIN_HOURS_BETWEEN_WATERING since the last watering -> water.
//   - MAX_DAYS_WITHOUT_WATER since the last watering -> water, reading or not.
//   - Watering = relay on for WATER_DURATION_MIN, then off.
//   - The counter is saved to flash hourly, so a power cut loses at most an
//     hour plus the outage.
//
// Safety (no endless watering):
//   - Relay off (reads WET) first thing on every boot; losing power closes
//     the NC contact, which also reads WET.
//   - Each session is hard-capped at 7 min (MAX_WATERING_SESSION_S), by loop()
//     and by an independent timer that forces the relay off.
//   - The counter resets when a session starts, so a reset mid-session can't
//     restart it; the watchdog resets the board if loop() stalls.
//   - Fit a 10k pull-down from the relay module's IN to GND so the relay stays
//     off in the fraction of a second before the firmware takes the pin.
//
// Logging: every line starts with [uptime d hh:mm:ss]. A HEARTBEAT line every
// HEARTBEAT_S shows the relay state, schedule and radio counters, so a quiet
// log still proves the board is alive. The LED blinks on each packet and stays
// on while watering.
//
// Serial commands (115200 baud): s = status, w = water now, x = stop watering,
// c = reset counter (as if just watered), f = jump counter to the fallback
// limit, h = help.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <Preferences.h>
#include <stdarg.h>
#include "config.h"

// XIAO ESP32-C6 RF switch: GPIO3 low enables it, GPIO14 picks the antenna.
#define RF_SWITCH_ENABLE_PIN 3
#define RF_ANTENNA_SELECT_PIN 14

// Hard cap on one watering session, whatever config.h says. Enforced twice:
// by loop(), and by an independent hardware-timer callback that forces the
// relay off even if loop() has stalled.
#define MAX_WATERING_SESSION_S (7 * 60)
static_assert(WATER_DURATION_MIN * 60 <= MAX_WATERING_SESSION_S,
              "WATER_DURATION_MIN must be 7 minutes or less");

static const uint32_t HOUR_S = 3600UL / TIME_SCALE;
static const uint32_t MIN_GAP_S = (uint32_t)MIN_HOURS_BETWEEN_WATERING * HOUR_S;
static const uint32_t MAX_GAP_S = (uint32_t)MAX_DAYS_WITHOUT_WATER * 24UL * HOUR_S;
static const uint32_t WATER_S = (uint32_t)WATER_DURATION_MIN * 60UL;
static const uint32_t SAVE_EVERY_S = HOUR_S;
static const uint32_t NO_READING_WARN_S = (uint32_t)NO_READING_WARN_H * HOUR_S;

static Preferences prefs;

// Uptime (s) of the last watering. Negative when it happened before this boot.
static int64_t lastWaterAt = 0;
static int64_t lastSaveAt = 0;
static uint32_t wateringCount = 0;

static bool watering = false;
static int64_t wateringStartedAt = 0;
static esp_timer_handle_t cutoffTimer = nullptr;
static volatile bool cutoffFired = false;
static int64_t lastProgressAt = 0;
static int64_t lastHeartbeatAt = 0;
static unsigned long ledBlinkUntilMs = 0;

// Mailbox between the ESP-NOW callback (Wi-Fi task) and loop().
static portMUX_TYPE packetMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool packetPending = false;
static MoisturePacket pendingPacket;
static int pendingRssi = 0;
// Packets the callback rejected, reported from loop().
static volatile uint32_t droppedWrongSize = 0;
static volatile uint32_t droppedWrongSender = 0;
static volatile int lastDroppedLen = 0;
static uint8_t lastDroppedMac[6];
static uint32_t reportedWrongSize = 0, reportedWrongSender = 0;

static bool radioOk = false;
static bool filterSender = false;

// Last reading, for status.
static bool haveReading = false;
static MoisturePacket lastPacket;
static int lastAverage = -1;  // -1 = no usable sensor in the last packet
static uint32_t lastSeq = 0;
static int lastRssi = 0;
static int64_t lastReadingAt = 0;
static uint32_t packetsReceived = 0;
static uint32_t packetsMissed = 0;
static uint32_t packetsDuplicate = 0;

// ---- Helpers ---------------------------------------------------------------

static int64_t uptimeS() {
  return esp_timer_get_time() / 1000000LL;
}

static uint32_t sinceWaterS() {
  int64_t s = uptimeS() - lastWaterAt;
  return s < 0 ? 0 : (uint32_t)s;
}

static float asHours(uint32_t s) {
  return (float)s / (float)HOUR_S;
}

// Timestamped log line: [d hh:mm:ss] message
static void logMsg(const char *fmt, ...) {
  char msg[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);
  uint32_t t = (uint32_t)uptimeS();
  Serial.printf("[%lud %02lu:%02lu:%02lu] %s\n", (unsigned long)(t / 86400),
                (unsigned long)(t / 3600 % 24), (unsigned long)(t / 60 % 60),
                (unsigned long)(t % 60), msg);
}

static String macToString(const uint8_t *mac) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

// Duration in real seconds as "1d 02h", "3h 04m" or "5m 06s".
static String fmtDuration(uint32_t s) {
  char buf[24];
  if (s >= 86400) snprintf(buf, sizeof(buf), "%lud %02luh", (unsigned long)(s / 86400), (unsigned long)(s / 3600 % 24));
  else if (s >= 3600) snprintf(buf, sizeof(buf), "%luh %02lum", (unsigned long)(s / 3600), (unsigned long)(s / 60 % 60));
  else snprintf(buf, sizeof(buf), "%lum %02lus", (unsigned long)(s / 60), (unsigned long)(s % 60));
  return String(buf);
}

static const char *resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "power on";
    case ESP_RST_EXT: return "external reset pin";
    case ESP_RST_SW: return "software restart";
    case ESP_RST_PANIC: return "crash (panic)";
    case ESP_RST_INT_WDT: return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog (loop stalled)";
    case ESP_RST_WDT: return "other watchdog";
    case ESP_RST_DEEPSLEEP: return "deep sleep wake";
    case ESP_RST_BROWNOUT: return "brownout (supply dipped)";
    case ESP_RST_USB: return "USB (upload or serial reset)";
    case ESP_RST_JTAG: return "JTAG";
    case ESP_RST_PWR_GLITCH: return "power glitch";
    case ESP_RST_CPU_LOCKUP: return "CPU lockup";
    default: return "other";
  }
}

// ---- Relay -----------------------------------------------------------------

static void setLed(bool on) {
  bool level = STATUS_LED_ACTIVE_LOW ? !on : on;
  digitalWrite(STATUS_LED_PIN, level ? HIGH : LOW);
}

static void writeRelayPin(bool dry) {
  bool level = RELAY_ACTIVE_HIGH ? dry : !dry;
  digitalWrite(RELAY_PIN, level ? HIGH : LOW);
}

static void setRelayDry(bool dry) {
  bool level = RELAY_ACTIVE_HIGH ? dry : !dry;
  writeRelayPin(dry);
  setLed(dry);
  logMsg("relay -> %s (pin %d driven %s, reads back %s)",
       dry ? "ENERGIZED: controller reads DRY" : "off: controller reads WET",
       RELAY_PIN, level ? "HIGH" : "LOW", digitalRead(RELAY_PIN) ? "HIGH" : "LOW");
}

// ---- Schedule --------------------------------------------------------------

static void saveCounter(bool announce) {
  uint32_t since = sinceWaterS();
  size_t written = prefs.putULong("since_s", since);
  lastSaveAt = uptimeS();
  if (written == 0) {
    logMsg("ERROR: saving counter to flash failed");
  } else if (announce) {
    logMsg("saved counter to flash: %.1f h since last watering", asHours(since));
  }
}

// Runs in the esp_timer task, independent of loop(): forces the relay off at
// the hard cap even if loop() is stuck. loop() then tidies up.
static void onCutoffTimer(void *) {
  writeRelayPin(false);
  cutoffFired = true;
}

static void startWatering(const char *reason) {
  if (watering) {
    logMsg("already watering; '%s' ignored", reason);
    return;
  }
  logMsg("WATERING START (%s): %.1f h since last watering, running %lu min (hard cap %d min)",
       reason, asHours(sinceWaterS()), (unsigned long)WATER_DURATION_MIN, MAX_WATERING_SESSION_S / 60);

  // Count the session as done before the relay turns on, and mark it in flash.
  // If the board resets mid-session (e.g. a brownout when the coil energizes),
  // it reboots with the relay off and won't water again until the next
  // allowed time, instead of restarting the session over and over.
  watering = true;
  wateringStartedAt = uptimeS();
  lastProgressAt = wateringStartedAt;
  lastWaterAt = wateringStartedAt;
  wateringCount++;
  prefs.putULong("count", wateringCount);
  prefs.putBool("session", true);
  saveCounter(false);

  cutoffFired = false;
  if (cutoffTimer) esp_timer_start_once(cutoffTimer, (uint64_t)MAX_WATERING_SESSION_S * 1000000ULL);
  setRelayDry(true);
}

static void stopWatering(bool completed) {
  if (!watering) {
    logMsg("not watering; nothing to stop");
    return;
  }
  watering = false;
  setRelayDry(false);
  if (cutoffTimer) esp_timer_stop(cutoffTimer);
  prefs.putBool("session", false);
  uint32_t ran = (uint32_t)(uptimeS() - wateringStartedAt);
  if (completed) {
    logMsg("WATERING DONE after %s (total waterings: %lu)",
         fmtDuration(ran).c_str(), (unsigned long)wateringCount);
  } else {
    logMsg("WATERING STOPPED early after %s; still counts as today's watering", fmtDuration(ran).c_str());
  }
  uint32_t since = sinceWaterS();
  logMsg("next: dry reading can water in %.1f h; forced watering in %.1f h",
       asHours(MIN_GAP_S - min(since, MIN_GAP_S)), asHours(MAX_GAP_S - min(since, MAX_GAP_S)));
}

// ---- Radio -----------------------------------------------------------------

static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != (int)sizeof(MoisturePacket)) {
    lastDroppedLen = len;
    droppedWrongSize = droppedWrongSize + 1;
    return;
  }
  if (filterSender && memcmp(info->src_addr, SENDER_MAC, 6) != 0) {
    portENTER_CRITICAL(&packetMux);
    memcpy(lastDroppedMac, info->src_addr, 6);
    droppedWrongSender = droppedWrongSender + 1;
    portEXIT_CRITICAL(&packetMux);
    return;
  }

  portENTER_CRITICAL(&packetMux);
  memcpy(&pendingPacket, data, sizeof(MoisturePacket));
  pendingRssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
  packetPending = true;
  portEXIT_CRITICAL(&packetMux);
}

static void reportDroppedPackets() {
  if (droppedWrongSize != reportedWrongSize) {
    reportedWrongSize = droppedWrongSize;
    logMsg("dropped packet: %d bytes, expected %d (sender running old firmware?) [total %lu]",
         lastDroppedLen, (int)sizeof(MoisturePacket), (unsigned long)reportedWrongSize);
  }
  if (droppedWrongSender != reportedWrongSender) {
    uint8_t mac[6];
    portENTER_CRITICAL(&packetMux);
    memcpy(mac, lastDroppedMac, 6);
    portEXIT_CRITICAL(&packetMux);
    reportedWrongSender = droppedWrongSender;
    logMsg("dropped packet from %s: not SENDER_MAC %s [total %lu]",
         macToString(mac).c_str(), macToString(SENDER_MAC).c_str(), (unsigned long)reportedWrongSender);
  }
}

static bool rawIsValid(uint16_t raw) {
  return raw >= RAW_VALID_MIN && raw <= RAW_VALID_MAX;
}

// Fitted, and reading in the plausible range.
static bool sensorUsed(const MoisturePacket &pkt, int i) {
  return SENSORS[i].fitted && rawIsValid(pkt.raw[i]);
}

// Raw ADC -> 0-100 % using this sensor's calibration.
static int rawToPercent(int i, uint16_t raw) {
  // Works whichever way round rawDry and rawWet are.
  long dry = SENSORS[i].rawDry, wet = SENSORS[i].rawWet;
  if (dry == wet) return 0;
  long pct = (dry - (long)raw) * 100L / (dry - wet);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (int)pct;
}

// Average moisture of the usable sensors, or -1 if there are none.
static int averageMoisture(const MoisturePacket &pkt) {
  int sum = 0, used = 0;
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (sensorUsed(pkt, i)) {
      sum += rawToPercent(i, pkt.raw[i]);
      used++;
    }
  }
  return used ? (sum + used / 2) / used : -1;
}

// Fitted sensors as " A0=42%(raw 1900)"; with includeUnfitted, the floating
// pins too (handy for checking a newly wired sensor before marking it fitted).
static String sensorsToString(const MoisturePacket &pkt, bool includeUnfitted) {
  String s;
  char buf[48];
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (SENSORS[i].fitted) {
      if (rawIsValid(pkt.raw[i])) {
        snprintf(buf, sizeof(buf), " A%d=%d%%(raw %u)", i, rawToPercent(i, pkt.raw[i]), pkt.raw[i]);
      } else {
        snprintf(buf, sizeof(buf), " A%d=FAULT(raw %u)", i, pkt.raw[i]);
      }
    } else if (includeUnfitted) {
      snprintf(buf, sizeof(buf), " A%d=unfitted(raw %u)", i, pkt.raw[i]);
    } else {
      continue;
    }
    s += buf;
  }
  return s;
}

static void handlePacket(const MoisturePacket &pkt, int rssi) {
#if BLINK_ON_PACKET
  if (!watering) {
    setLed(true);
    ledBlinkUntilMs = millis() + 80;
  }
#endif

  const char *seqNote = "";
  if (packetsReceived > 0) {
    if (pkt.seq == lastSeq) {
      packetsDuplicate++;
      logMsg("RX duplicate seq=%lu ignored (a retry after a lost ACK)", (unsigned long)pkt.seq);
      return;
    }
    if (pkt.seq < lastSeq) {
      seqNote = "  [sender restarted: lights came on]";
    } else if (pkt.seq > lastSeq + 1) {
      packetsMissed += pkt.seq - lastSeq - 1;
      seqNote = "  [missed packets]";
    }
  } else {
    seqNote = "  [first packet since boot]";
  }

  uint32_t gap = haveReading ? (uint32_t)(uptimeS() - lastReadingAt) : 0;
  packetsReceived++;
  int avg = averageMoisture(pkt);
  haveReading = true;
  lastPacket = pkt;
  lastAverage = avg;
  lastSeq = pkt.seq;
  lastRssi = rssi;
  lastReadingAt = uptimeS();

  char avgStr[12];
  if (avg >= 0) snprintf(avgStr, sizeof(avgStr), "%d%%", avg);
  else snprintf(avgStr, sizeof(avgStr), "n/a");
  logMsg("RX seq=%lu avg=%s%s rssi=%d dBm%s%s%s", (unsigned long)pkt.seq, avgStr,
       sensorsToString(pkt, false).c_str(), rssi, seqNote,
       gap > 0 ? "  gap " : "", gap > 0 ? fmtDuration(gap).c_str() : "");
  if (rssi < -85) logMsg("  weak signal (%d dBm); packets may be lost at this range", rssi);

  if (watering) {
    logMsg("  -> already watering");
    return;
  }

  uint32_t since = sinceWaterS();
  if (avg < 0) {
    logMsg("  -> no usable sensor, no decision (forced watering in %.1f h still applies)",
         asHours(MAX_GAP_S - min(since, MAX_GAP_S)));
  } else if (avg >= DRY_THRESHOLD_PCT) {
    logMsg("  -> moist (%d%% >= %d%%), no watering", avg, DRY_THRESHOLD_PCT);
  } else if (since < MIN_GAP_S) {
    logMsg("  -> dry (%d%% < %d%%) but watered %.1f h ago; allowed again in %.1f h",
         avg, DRY_THRESHOLD_PCT, asHours(since), asHours(MIN_GAP_S - since));
  } else {
    logMsg("  -> dry (%d%% < %d%%) and %.1f h since last watering", avg, DRY_THRESHOLD_PCT, asHours(since));
    startWatering("soil dry");
  }
}

// ---- Status output ---------------------------------------------------------

static void heartbeat() {
  uint32_t since = sinceWaterS();
  String relay = watering
    ? "WATERING (" + fmtDuration(WATER_S - (uint32_t)(uptimeS() - wateringStartedAt)) + " left)"
    : "WET";
  String allowed = since >= MIN_GAP_S ? "now" : "in " + String(asHours(MIN_GAP_S - since), 1) + " h";
  String reading = haveReading
    ? (lastAverage >= 0 ? String(lastAverage) + "%" : String("n/a")) + " " + fmtDuration((uint32_t)(uptimeS() - lastReadingAt)) + " ago"
    : "none since boot";
  logMsg("HEARTBEAT relay=%s | since water %.1f h, dry-water %s, forced in %.1f h | last reading %s | rx %lu missed %lu dup %lu dropped %lu%s",
       relay.c_str(), asHours(since), allowed.c_str(), asHours(MAX_GAP_S - min(since, MAX_GAP_S)),
       reading.c_str(), (unsigned long)packetsReceived, (unsigned long)packetsMissed,
       (unsigned long)packetsDuplicate, (unsigned long)(droppedWrongSize + droppedWrongSender),
       radioOk ? "" : " | RADIO DOWN");

  uint32_t silentFor = haveReading ? (uint32_t)(uptimeS() - lastReadingAt) : (uint32_t)uptimeS();
  if (silentFor >= NO_READING_WARN_S) {
    logMsg("WARNING: no reading for %.1f h; check the sender (lights, power, range, MAC)", asHours(silentFor));
  }
}

static void printStatus() {
  uint32_t since = sinceWaterS();
  Serial.println("--- status ---");
  Serial.printf("uptime: %s   MAC: %s   channel: %d   radio: %s\n",
                fmtDuration((uint32_t)uptimeS()).c_str(), WiFi.macAddress().c_str(), ESPNOW_CHANNEL,
                radioOk ? "OK" : "FAILED");
  Serial.printf("relay: %s (pin %d reads %s)\n",
                watering ? "ENERGIZED, controller reads DRY (watering)" : "off, controller reads WET",
                RELAY_PIN, digitalRead(RELAY_PIN) ? "HIGH" : "LOW");
  if (watering) {
    Serial.printf("watering remaining: %s\n",
                  fmtDuration(WATER_S - (uint32_t)(uptimeS() - wateringStartedAt)).c_str());
  }
  Serial.printf("since last watering: %.1f h   (dry-watering allowed after %d h, forced at %d days = %lu h)\n",
                asHours(since), MIN_HOURS_BETWEEN_WATERING, MAX_DAYS_WITHOUT_WATER,
                (unsigned long)MAX_DAYS_WITHOUT_WATER * 24UL);
  Serial.printf("total waterings: %lu\n", (unsigned long)wateringCount);
  if (haveReading) {
    Serial.printf("last reading: avg=%d%%%s seq=%lu rssi=%d dBm, %s ago\n",
                  lastAverage, sensorsToString(lastPacket, true).c_str(), (unsigned long)lastSeq, lastRssi,
                  fmtDuration((uint32_t)(uptimeS() - lastReadingAt)).c_str());
  } else {
    Serial.println("last reading: none since boot");
  }
  Serial.printf("packets: %lu received, %lu missed, %lu duplicate, %lu wrong size, %lu wrong sender\n",
                (unsigned long)packetsReceived, (unsigned long)packetsMissed, (unsigned long)packetsDuplicate,
                (unsigned long)droppedWrongSize, (unsigned long)droppedWrongSender);
  if (TIME_SCALE != 1) Serial.printf("TIME_SCALE=%d: 1 h = %lus\n", TIME_SCALE, (unsigned long)HOUR_S);
  Serial.println("--------------");
}

static void printSettings() {
  logMsg("settings: water %d min | dry below %d%% | min gap %d h | forced after %d days",
       WATER_DURATION_MIN, DRY_THRESHOLD_PCT, MIN_HOURS_BETWEEN_WATERING, MAX_DAYS_WITHOUT_WATER);
  int fitted = 0;
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (!SENSORS[i].fitted) continue;
    fitted++;
    logMsg("settings: sensor A%d fitted, dry=%u wet=%u (valid %d-%d); %d%% = raw %ld",
           i, SENSORS[i].rawDry, SENSORS[i].rawWet, RAW_VALID_MIN, RAW_VALID_MAX, DRY_THRESHOLD_PCT,
           (long)SENSORS[i].rawDry - ((long)SENSORS[i].rawDry - SENSORS[i].rawWet) * DRY_THRESHOLD_PCT / 100);
  }
  if (fitted == 0) logMsg("WARNING: no sensors marked fitted; only the fallback timer will water");
  logMsg("settings: relay pin %d active-%s | heartbeat %ds | TIME_SCALE %d (1 h = %lus)",
       RELAY_PIN, RELAY_ACTIVE_HIGH ? "high" : "low", HEARTBEAT_S, TIME_SCALE, (unsigned long)HOUR_S);
  if (TIME_SCALE != 1) logMsg("WARNING: TIME_SCALE is not 1; schedule is sped up for testing");
}

static void printHelp() {
  Serial.println("commands: s=status  w=water now  x=stop watering  c=reset counter  f=force fallback  r=relay test  h=help");
}

// Bench test: toggle the relay every RELAY_TEST_S seconds so the fake-probe resistance can
// be checked with a meter. Runs until any key; the schedule pauses meanwhile.
static void relayTest() {
  if (watering) {
    logMsg("relay test refused: watering in progress (x to stop it first)");
    return;
  }
  logMsg("RELAY TEST: toggling every %d s; send any key to stop", RELAY_TEST_S);
  while (Serial.available()) Serial.read();
  bool dry = false;
  int cycles = 0;
  while (!Serial.available()) {
    dry = !dry;
    setRelayDry(dry);
    if (!dry) cycles++;
    for (int i = 0; i < RELAY_TEST_S * 100 && !Serial.available(); i++) {
      esp_task_wdt_reset();
      delay(10);
    }
  }
  while (Serial.available()) Serial.read();
  setRelayDry(false);
  logMsg("RELAY TEST done after %d cycles; relay off (controller reads WET)", cycles);
}

static void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'r': relayTest(); break;
      case 's': printStatus(); break;
      case 'w': logMsg("command: water now"); startWatering("manual"); break;
      case 'x': logMsg("command: stop watering"); stopWatering(false); break;
      case 'c':
        lastWaterAt = uptimeS();
        saveCounter(false);
        logMsg("command: counter reset to 0 (as if just watered)");
        break;
      case 'f':
        lastWaterAt = uptimeS() - (int64_t)MAX_GAP_S;
        saveCounter(false);
        logMsg("command: counter set to the fallback limit; forced watering next");
        break;
      case 'h': case '?': printHelp(); break;
      default: break;
    }
  }
}

// ---- Setup -----------------------------------------------------------------

static void setupWatchdog() {
  esp_task_wdt_config_t cfg = {
    .timeout_ms = WATCHDOG_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  esp_err_t err = esp_task_wdt_reconfigure(&cfg);
  if (err != ESP_OK) err = esp_task_wdt_init(&cfg);
  if (err == ESP_OK) err = esp_task_wdt_add(NULL);  // setup() and loop() run in the same task
  if (err == ESP_OK) logMsg("watchdog: armed, %d s", WATCHDOG_TIMEOUT_S);
  else logMsg("WARNING: watchdog setup failed (%d)", err);
}

static bool setupRadio() {
  pinMode(RF_SWITCH_ENABLE_PIN, OUTPUT);
  digitalWrite(RF_SWITCH_ENABLE_PIN, LOW);
  pinMode(RF_ANTENNA_SELECT_PIN, OUTPUT);
  digitalWrite(RF_ANTENNA_SELECT_PIN, USE_EXTERNAL_ANTENNA ? HIGH : LOW);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  uint8_t primary = 0;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&primary, &second);
  logMsg("radio: MAC %s, channel %d (wanted %d), %s antenna",
       WiFi.macAddress().c_str(), primary, ESPNOW_CHANNEL, USE_EXTERNAL_ANTENNA ? "external" : "on-board");
  logMsg("       put this MAC in sender/config.h RECEIVER_MAC");

  esp_err_t err = esp_now_init();
  if (err != ESP_OK) {
    logMsg("ERROR: ESP-NOW init failed (%d)", err);
    return false;
  }
  esp_now_register_recv_cb(onRecv);
  logMsg("radio: ESP-NOW listening, expecting %d-byte packets", (int)sizeof(MoisturePacket));
  return true;
}

void setup() {
  // Relay off (controller reads wet) before anything else, on every boot,
  // whatever state it was in before the reset or power loss.
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_HIGH ? LOW : HIGH);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  setLed(false);

  Serial.begin(115200);
  unsigned long waitStart = millis();
  while (!Serial && millis() - waitStart < 1000) delay(10);

  Serial.println();
  Serial.println("=== Moisture receiver ===");
  esp_reset_reason_t reason = esp_reset_reason();
  logMsg("boot: reset reason = %s (%d)", resetReasonName(reason), (int)reason);
  if (reason == ESP_RST_TASK_WDT || reason == ESP_RST_PANIC || reason == ESP_RST_BROWNOUT) {
    logMsg("WARNING: last run ended abnormally; relay was released by the reset");
  }
  setRelayDry(false);
  printSettings();

  prefs.begin("moisture", false);
  uint32_t since;
  if (prefs.isKey("since_s")) {
    since = prefs.getULong("since_s", 0);
    logMsg("flash: restored counter, %.1f h since last watering", asHours(since));
  } else {
    // First boot: allow a watering on the first dry reading.
    since = MIN_GAP_S;
    logMsg("flash: no saved counter (first boot); starting at %.1f h so a dry reading can water",
         asHours(since));
  }
  wateringCount = prefs.getULong("count", 0);
  logMsg("flash: %lu waterings so far", (unsigned long)wateringCount);
  if (prefs.getBool("session", false)) {
    // The counter was already reset when that session started, so it won't
    // simply restart now.
    logMsg("WARNING: a watering session was cut short by a reset; it still counts, not repeating it");
    prefs.putBool("session", false);
  }
  lastWaterAt = uptimeS() - (int64_t)since;
  saveCounter(false);

  esp_timer_create_args_t timerArgs = {};
  timerArgs.callback = onCutoffTimer;
  timerArgs.name = "water_cutoff";
  if (esp_timer_create(&timerArgs, &cutoffTimer) != ESP_OK) {
    cutoffTimer = nullptr;
    logMsg("WARNING: safety cutoff timer unavailable; loop() still enforces the cap");
  }

  setupWatchdog();

  radioOk = setupRadio();
  if (!radioOk) {
    // Keep running: the fallback timer still waters without the radio.
    logMsg("running on the fallback timer only");
  }

  static const uint8_t ANY[6] = {0, 0, 0, 0, 0, 0};
  filterSender = memcmp(SENDER_MAC, ANY, 6) != 0;
  if (filterSender) logMsg("radio: accepting packets only from %s", macToString(SENDER_MAC).c_str());
  else logMsg("radio: SENDER_MAC not set; accepting packets from any sender");

  logMsg("ready");
  printStatus();
  printHelp();
  lastHeartbeatAt = uptimeS();
}

void loop() {
  esp_task_wdt_reset();

  if (packetPending) {
    MoisturePacket pkt;
    int rssi;
    portENTER_CRITICAL(&packetMux);
    pkt = pendingPacket;
    rssi = pendingRssi;
    packetPending = false;
    portEXIT_CRITICAL(&packetMux);
    handlePacket(pkt, rssi);
  }
  reportDroppedPackets();

  if (ledBlinkUntilMs && (long)(millis() - ledBlinkUntilMs) >= 0) {
    ledBlinkUntilMs = 0;
    setLed(watering);
  }

  int64_t now = uptimeS();

  if (watering) {
    if (cutoffFired) {
      logMsg("SAFETY CUTOFF: hardware timer ended the session at the %d min cap", MAX_WATERING_SESSION_S / 60);
      stopWatering(false);
    } else if (now - wateringStartedAt >= (int64_t)min(WATER_S, (uint32_t)MAX_WATERING_SESSION_S)) {
      stopWatering(true);
    } else if (now - lastProgressAt >= WATERING_PROGRESS_S) {
      lastProgressAt = now;
      logMsg("watering: %s left", fmtDuration(WATER_S - (uint32_t)(now - wateringStartedAt)).c_str());
    }
  }

  if (!watering && sinceWaterS() >= MAX_GAP_S) {
    startWatering("fallback: max days without water");
  }

  if (now - lastSaveAt >= (int64_t)SAVE_EVERY_S) {
    saveCounter(true);
  }

  if (now - lastHeartbeatAt >= HEARTBEAT_S) {
    lastHeartbeatAt = now;
    heartbeat();
  }

  handleSerial();
  delay(10);
}
