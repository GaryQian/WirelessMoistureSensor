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
//     RAW_VALID_MIN-RAW_VALID_MAX as faulty. Each sensor's readings are
//     averaged over DECISION_WINDOW_S and one decision is made per window.
//   - The controller only checks the probe every CHECK_EVERY_H hours from
//     local midnight, so watering is queued and played back WATER_LEAD_S
//     before its next check (23:59:30, 03:59:30, ... in LOCAL_TZ): relay DRY
//     for WATER_DURATION_MIN, then off. It counts as a watering only once that
//     hold completes.
//   - Average < DRY_THRESHOLD_PCT and at least MIN_HOURS_BETWEEN_WATERING
//     since the last watering -> queue. A later moist window cancels it.
//   - MAX_DAYS_WITHOUT_WATER since the last watering -> queue, reading or not,
//     retried at every check until one completes.
//   - One attempt per check. A failed attempt (stopped, reset) discards the
//     queued signal and the current window; only new readings can queue again.
//   - The last watering is saved to flash as UTC, so time powered off counts.
//     Before the clock is set, a since-watering counter saved hourly is used.
//   - The clock comes from TIME_API_URL (NTP_SERVER as fallback) every
//     CLOCK_RESYNC_H.
//
// Network: the sender is fixed on ESPNOW_CHANNEL and the radio can only be on
// one channel, so the receiver stays on Wi-Fi only while the router is on that
// channel. Then it serves a read-only web page with a chart on WEB_PORT, with
// JSON at /api/current and /api/history (also http://MDNS_NAME.local/ at home),
// and accepts OTA uploads if OTA_PASSWORD is in secrets.h. The server runs in
// its own task on a copy of the state, so web clients can't stall the watering
// logic. If the router is on another channel, the receiver stays off Wi-Fi
// (web page down) and syncs the clock by joining for a few seconds at a time.
//
// Safety (no endless watering):
//   - Relay off (reads WET) first thing on every boot; losing power closes
//     the NC contact, which also reads WET.
//   - Each session is hard-capped at 7 min (MAX_WATERING_SESSION_S), by loop()
//     and by an independent timer that forces the relay off.
//   - The check of each attempt is saved to flash before the relay turns on,
//     so a reset mid-session can't repeat it; the watchdog resets the board if
//     loop() stalls.
//   - Fit a 10k pull-down from the relay module's IN to GND so the relay stays
//     off in the fraction of a second before the firmware takes the pin.
//
// Logging: every line starts with [uptime d hh:mm:ss], plus UTC hh:mm:ssZ once
// the clock is set. A HEARTBEAT line every
// HEARTBEAT_S shows the relay state, schedule and radio counters, so a quiet
// log still proves the board is alive. The LED blinks on each packet and stays
// on while watering.
//
// Serial commands (115200 baud): s = status, w = water now, x = stop watering
// (or cancel a queued one),
// c = reset counter (as if just watered), f = jump counter to the fallback
// limit, p = queue a watering for the next slot, t = sync clock now, l = list
// moisture history, n = scan for the Wi-Fi network on all channels, r = relay
// test, d = hold relay DRY until any key, h = help.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <NetworkClientSecure.h>
#include <esp_sntp.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <time.h>
#include <stdarg.h>
#include "config.h"
#include "web.h"
#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Copy receiver/secrets.example.h to receiver/secrets.h and set the Wi-Fi name and password"
#endif

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
static_assert(CHECK_EVERY_H > 0 && 24 % CHECK_EVERY_H == 0, "CHECK_EVERY_H must divide 24");
static_assert(WATER_LEAD_S > 0 && WATER_LEAD_S < WATER_DURATION_MIN * 60,
              "the DRY hold must start before the check and last past it");

static Preferences prefs;

// Uptime (s) of the last watering. Negative when it happened before this boot.
static int64_t lastWaterAt = 0;
static int64_t lastSaveAt = 0;
static uint32_t wateringCount = 0;

// UTC = uptimeS() + utcOffset, valid once clockSynced.
static bool clockSynced = false;
static int64_t utcOffset = 0;
static bool syncAttempted = false;
static bool lastSyncOk = false;
static int64_t lastSyncAttemptAt = 0;
static int64_t lastSyncOkAt = 0;
static const char *clockSource = "";
// Last watering (UTC) read from flash at boot, applied when the clock is set; 0 = none.
static int64_t restoredLastWaterUtc = 0;

static PendingReason pending = PENDING_NONE;
// UTC time of the controller check whose slot was last used; -1 = none.
static int64_t lastAttemptCheck = -1;

static bool watering = false;
static PendingReason wateringReason = PENDING_NONE;
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
static bool wifiUp = false;
static bool wifiJoining = false;
static int64_t wifiJoinStartedAt = 0;
static int64_t lastWifiCheckAt = -(int64_t)WIFI_RETRY_S;
static bool routerOnEspNowChannel = true;
static bool networkServicesStarted = false;

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

// Readings collected toward the next watering decision.
static int64_t windowStartedAt = 0;
static uint32_t windowPackets = 0;
static uint32_t windowSum[MAX_SENSORS];
static uint32_t windowValid[MAX_SENSORS];
static MoisturePacket windowLastPacket;

// Last closed window, for the web page.
static bool haveWindow = false;
static MoisturePacket lastWindowPacket;
static int lastWindowAvgAny = -1;
static int64_t lastWindowClosedAt = 0;

// ---- Helpers ---------------------------------------------------------------

static int64_t uptimeS() {
  return esp_timer_get_time() / 1000000LL;
}

static int64_t nowUtc() {
  return uptimeS() + utcOffset;
}

static String fmtUtc(int64_t utc, bool withDate) {
  time_t t = (time_t)utc;
  struct tm tm;
  gmtime_r(&t, &tm);
  char buf[24];
  strftime(buf, sizeof(buf), withDate ? "%Y-%m-%d %H:%M:%SZ" : "%H:%M:%SZ", &tm);
  return String(buf);
}

// configTime() overwrites TZ, so this is reapplied after every NTP sync.
static void applyLocalTz() {
  setenv("TZ", LOCAL_TZ, 1);
  tzset();
}

static struct tm localTm(int64_t utc) {
  time_t t = (time_t)utc;
  struct tm tm;
  localtime_r(&t, &tm);
  return tm;
}

static String fmtLocal(int64_t utc) {
  struct tm tm = localTm(utc);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S %Z", &tm);
  return String(buf);
}

static uint32_t sinceWaterS() {
  int64_t s = uptimeS() - lastWaterAt;
  return s < 0 ? 0 : (uint32_t)s;
}

static float asHours(uint32_t s) {
  return (float)s / (float)HOUR_S;
}

// Timestamped log line: [d hh:mm:ss] or [d hh:mm:ss hh:mm:ssZ] message
static void logMsg(const char *fmt, ...) {
  char msg[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);
  uint32_t t = (uint32_t)uptimeS();
  Serial.printf("[%lud %02lu:%02lu:%02lu%s%s] %s\n", (unsigned long)(t / 86400),
                (unsigned long)(t / 3600 % 24), (unsigned long)(t / 60 % 60),
                (unsigned long)(t % 60), clockSynced ? " " : "",
                clockSynced ? fmtUtc(nowUtc(), false).c_str() : "", msg);
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

// lastWaterChanged: lastWaterAt was just set (watering, 'c', 'f'), so a UTC
// time restored from flash at boot no longer applies.
static void saveCounter(bool announce, bool lastWaterChanged) {
  uint32_t since = sinceWaterS();
  size_t written = prefs.putULong("since_s", since);
  if (clockSynced) {
    if (prefs.putLong64("water_utc", nowUtc() - since) == 0) written = 0;
  } else if (lastWaterChanged) {
    prefs.remove("water_utc");
  }
  if (lastWaterChanged) restoredLastWaterUtc = 0;
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

static void discardWindow();
static void recordWateringEvent(bool completed);

static void startWatering(PendingReason reason) {
  if (watering) {
    logMsg("already watering; '%s' ignored", pendingName(reason));
    return;
  }
  logMsg("WATERING START (%s): %.1f h since last watering, relay DRY for %lu min (hard cap %d min)",
       pendingName(reason), asHours(sinceWaterS()), (unsigned long)WATER_DURATION_MIN, MAX_WATERING_SESSION_S / 60);

  watering = true;
  wateringReason = reason;
  wateringStartedAt = uptimeS();
  lastProgressAt = wateringStartedAt;
  prefs.putBool("session", true);

  cutoffFired = false;
  if (cutoffTimer) esp_timer_start_once(cutoffTimer, (uint64_t)MAX_WATERING_SESSION_S * 1000000ULL);
  setRelayDry(true);
}

// completed: the DRY hold ran its full length, so it counts as a watering.
// Either way the queued signal and the current window are discarded, so only
// fresh readings can queue the next one.
static void stopWatering(bool completed) {
  if (!watering) {
    logMsg("not watering; nothing to stop");
    return;
  }
  watering = false;
  setRelayDry(false);
  if (cutoffTimer) esp_timer_stop(cutoffTimer);
  prefs.putBool("session", false);
  pending = PENDING_NONE;
  discardWindow();
  recordWateringEvent(completed);
  uint32_t ran = (uint32_t)(uptimeS() - wateringStartedAt);
  if (completed) {
    lastWaterAt = uptimeS();
    wateringCount++;
    prefs.putULong("count", wateringCount);
    saveCounter(false, true);
    logMsg("WATERING DONE after %s; counted as watered (total waterings: %lu)",
         fmtDuration(ran).c_str(), (unsigned long)wateringCount);
  } else {
    logMsg("WATERING FAILED: stopped after %s; not counted, signal discarded, waiting for new readings",
         fmtDuration(ran).c_str());
  }
  uint32_t since = sinceWaterS();
  logMsg("next: dry reading can queue in %.1f h; forced watering queued in %.1f h",
       asHours(MIN_GAP_S - min(since, MIN_GAP_S)), asHours(MAX_GAP_S - min(since, MAX_GAP_S)));
}

// ---- Clock -----------------------------------------------------------------

// Days since 1970-01-01 for a proleptic Gregorian date (Hinnant's days_from_civil).
static int64_t daysFromCivil(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

// "2026-10-02T09:17:04.1048446Z" -> seconds since the epoch.
static bool parseIsoUtc(const char *s, int64_t &utc) {
  int y, mo, d, h, mi, sec;
  if (sscanf(s, "%4d-%2d-%2dT%2d:%2d:%2d", &y, &mo, &d, &h, &mi, &sec) != 6) return false;
  if (y < 2024 || mo < 1 || mo > 12 || d < 1 || d > 31 || h > 23 || mi > 59 || sec > 60) return false;
  utc = daysFromCivil(y, mo, d) * 86400LL + h * 3600LL + mi * 60LL + sec;
  return true;
}

static bool fetchTimeApi(int64_t &utc) {
  NetworkClientSecure client;
  client.setInsecure();
  client.setHandshakeTimeout(10);
  HTTPClient http;
  http.setConnectTimeout(5000);
  http.setTimeout(5000);
  if (!http.begin(client, TIME_API_URL)) {
    logMsg("clock: timeapi request setup failed");
    return false;
  }
  int code = http.GET();
  esp_task_wdt_reset();
  String body = code == HTTP_CODE_OK ? http.getString() : String();
  http.end();
  if (code != HTTP_CODE_OK) {
    logMsg("clock: timeapi failed (HTTP %d %s)", code, code < 0 ? HTTPClient::errorToString(code).c_str() : "");
    return false;
  }
  const char *key = "\"utc_time\":\"";
  int at = body.indexOf(key);
  if (at < 0 || !parseIsoUtc(body.c_str() + at + strlen(key), utc)) {
    logMsg("clock: timeapi reply not understood: %.80s", body.c_str());
    return false;
  }
  return true;
}

static bool fetchNtp(int64_t &utc) {
  configTime(0, 0, NTP_SERVER);
  bool ok = false;
  for (int i = 0; i < 500 && !ok; i++) {
    esp_task_wdt_reset();
    delay(10);
    ok = esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED;
  }
  if (ok) utc = (int64_t)time(nullptr);
  esp_sntp_stop();
  applyLocalTz();
  if (!ok) logMsg("clock: NTP (%s) gave no answer", NTP_SERVER);
  return ok;
}

static void applyClock(int64_t utc, const char *source) {
  bool first = !clockSynced;
  int64_t drift = clockSynced ? utc - nowUtc() : 0;
  utcOffset = utc - uptimeS();
  clockSynced = true;
  clockSource = source;
  lastSyncOkAt = uptimeS();
  if (first) logMsg("clock: set from %s: %s", source, fmtUtc(utc, true).c_str());
  else logMsg("clock: resynced from %s (was %+lld s off)", source, (long long)drift);

  if (restoredLastWaterUtc > 0) {
    int64_t since = utc - restoredLastWaterUtc;
    if (since >= 0) {
      lastWaterAt = uptimeS() - since;
      logMsg("flash: last watering was %s, %.1f h ago (includes time powered off)",
           fmtUtc(restoredLastWaterUtc, true).c_str(), asHours((uint32_t)since));
    } else {
      logMsg("flash: saved last watering %s is in the future; keeping the counter",
           fmtUtc(restoredLastWaterUtc, true).c_str());
    }
    restoredLastWaterUtc = 0;
  }
  saveCounter(false, false);
}

static void restoreEspNowChannel() {
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
}

// Joins Wi-Fi on whatever channel the router uses, fetches the time, leaves,
// and returns to ESPNOW_CHANNEL. Packets sent meanwhile are lost.
static const char *fetchTimeViaBriefJoin(int64_t &utc) {
  logMsg("clock: joining Wi-Fi \"%s\" briefly to fetch the time", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_JOIN_TIMEOUT_S * 1000UL) {
    esp_task_wdt_reset();
    delay(50);
  }
  const char *source = nullptr;
  if (WiFi.status() == WL_CONNECTED) {
    logMsg("clock: Wi-Fi up after %lu ms (channel %d, %d dBm)", millis() - start, WiFi.channel(), WiFi.RSSI());
    esp_task_wdt_reset();
    if (fetchTimeApi(utc)) source = "timeapi.io";
    else if (fetchNtp(utc)) source = "NTP";
  } else {
    logMsg("clock: could not join Wi-Fi within %d s (status %d)", WIFI_JOIN_TIMEOUT_S, (int)WiFi.status());
  }
  WiFi.disconnect();
  for (int i = 0; i < 50 && WiFi.status() == WL_CONNECTED; i++) delay(10);
  restoreEspNowChannel();
  return source;
}

// Blocks for up to ~40 s, feeding the watchdog.
static void syncClock() {
  syncAttempted = true;
  lastSyncAttemptAt = uptimeS();
  int64_t utc = 0;
  const char *source = nullptr;
  if (wifiUp) {
    if (fetchTimeApi(utc)) source = "timeapi.io";
    else if (fetchNtp(utc)) source = "NTP";
  } else {
    source = fetchTimeViaBriefJoin(utc);
  }

  lastSyncOk = source != nullptr;
  if (lastSyncOk) applyClock(utc, source);
  else logMsg("clock: sync failed; retrying in %d min%s", CLOCK_RETRY_MIN,
              clockSynced ? " (keeping the current clock)" : "");
}

// First controller check strictly after utc. UTC hours line up with LOCAL_TZ
// hours (whole-hour offsets), so only whole UTC hours need testing.
static int64_t checkAfter(int64_t utc) {
  int64_t t = (utc / 3600 + 1) * 3600;
  for (int i = 0; i < 48 && localTm(t).tm_hour % CHECK_EVERY_H != 0; i++) t += 3600;
  return t;
}

static bool nearSlot() {
  if (!clockSynced) return false;
  int64_t utc = nowUtc();
  return checkAfter(utc) - utc <= WATER_LEAD_S + 120;
}

static void maybeSyncClock() {
  if (wifiJoining || watering) return;
  if (syncAttempted) {
    int64_t wait = lastSyncOk ? (int64_t)CLOCK_RESYNC_H * 3600 : (int64_t)CLOCK_RETRY_MIN * 60;
    if (uptimeS() - lastSyncAttemptAt < wait) return;
  }
  if (nearSlot()) return;
  syncClock();
}

// ---- Watering slot ---------------------------------------------------------

// UTC start of the next slot (WATER_LEAD_S before a check) that can still be used.
static int64_t nextSlotUtc() {
  int64_t check = checkAfter(nowUtc());
  if (check == lastAttemptCheck) check = checkAfter(check);
  return check - WATER_LEAD_S;
}

static String slotDescription() {
  if (!clockSynced) return "waiting for the clock";
  int64_t slot = nextSlotUtc();
  int64_t wait = slot - nowUtc();
  return fmtLocal(slot) + (wait > 0 ? ", in " + fmtDuration((uint32_t)wait) : String(", now"));
}

static void setPending(PendingReason r) {
  if (pending == r) return;
  pending = r;
  if (r != PENDING_NONE) logMsg("  -> QUEUED (%s) for the next slot: %s", pendingName(r), slotDescription().c_str());
}

static void runSlot() {
  if (pending == PENDING_NONE || watering || !clockSynced) return;
  int64_t utc = nowUtc();
  int64_t check = checkAfter(utc);
  if (check - utc > WATER_LEAD_S || check == lastAttemptCheck) return;

  lastAttemptCheck = check;
  prefs.putLong64("attempt_chk", lastAttemptCheck);
  PendingReason reason = pending;
  pending = PENDING_NONE;
  if (reason == PENDING_DRY && sinceWaterS() < MIN_GAP_S) {
    logMsg("slot: queued dry signal dropped; watered %.1f h ago", asHours(sinceWaterS()));
    return;
  }
  logMsg("slot: playing back the queued watering for the %s check", fmtLocal(check).c_str());
  startWatering(reason);
}

// ---- Flash records ---------------------------------------------------------

// Arrays of fixed-size records, oldest first, each starting with a uint32_t
// UTC time, saved whole under one Preferences key.

static int loadRecords(const char *key, void *out, size_t size, int max) {
  size_t len = prefs.isKey(key) ? prefs.getBytesLength(key) : 0;
  if (len == 0) return 0;
  if (len % size != 0) {
    logMsg("WARNING: saved %s is corrupt (%u bytes); starting over", key, (unsigned)len);
    return 0;
  }
  uint8_t *saved = (uint8_t *)malloc(len);
  if (!saved) return 0;
  prefs.getBytes(key, saved, len);
  int n = len / size;
  int skip = n > max ? n - max : 0;
  memcpy(out, saved + skip * size, (n - skip) * size);
  free(saved);
  return n - skip;
}

// Drops records older than keepAfter, then the oldest beyond max - 1, and appends rec.
static void appendRecord(void *buf, int &count, size_t size, int max, const void *rec, int64_t keepAfter) {
  uint8_t *b = (uint8_t *)buf;
  int drop = 0;
  uint32_t utc;
  while (drop < count && (memcpy(&utc, b + drop * size, 4), (int64_t)utc < keepAfter)) drop++;
  if (count - drop >= max) drop = count - max + 1;
  memmove(b, b + drop * size, (count - drop) * size);
  count -= drop;
  memcpy(b + count * size, rec, size);
  count++;
}

static bool saveRecords(const char *key, const void *buf, int count, size_t size) {
  if (prefs.putBytes(key, buf, count * size) != 0) return true;
  logMsg("ERROR: saving %s to flash failed", key);
  return false;
}

// ---- History ---------------------------------------------------------------

static const int64_t HISTORY_KEEP_S = (int64_t)HISTORY_DAYS * 86400;

// Oldest first.
static HistoryEntry history[HISTORY_MAX];
static int historyCount = 0;
// Half-day (midnight-noon, noon-midnight in LOCAL_TZ) last checked; -1 = not yet.
static int64_t historyHalfDay = -1;
// Latest window average and its UTC time; lastWindowAt 0 = none.
static int lastWindowAvg = -1;
static int64_t lastWindowAt = 0;

static int64_t halfDayOf(int64_t utc) {
  struct tm tm = localTm(utc);
  return ((int64_t)tm.tm_year * 366 + tm.tm_yday) * 2 + (tm.tm_hour >= 12 ? 1 : 0);
}

static void loadHistory() {
  prefs.remove("history");
  historyCount = loadRecords("history2", history, sizeof(HistoryEntry), HISTORY_MAX);
  if (historyCount) logMsg("flash: %d moisture history entries restored", historyCount);
}

static void saveHistory(int64_t utc) {
  HistoryEntry e = {(uint32_t)utc, (uint32_t)lastWindowAt, (int8_t)lastWindowAvg};
  appendRecord(history, historyCount, sizeof(HistoryEntry), HISTORY_MAX, &e, utc - HISTORY_KEEP_S);
  if (saveRecords("history2", history, historyCount, sizeof(HistoryEntry))) {
    logMsg("history: saved %d%% measured %s (%d of %d entries)", lastWindowAvg,
         fmtLocal(lastWindowAt).c_str(), historyCount, HISTORY_MAX);
  }
}

static void historyTick() {
  if (!clockSynced) return;
  int64_t utc = nowUtc();
  int64_t half = halfDayOf(utc);
  if (half == historyHalfDay) return;
  bool first = historyHalfDay < 0;
  historyHalfDay = half;
  if (first) return;

  int64_t lastSaved = historyCount ? (int64_t)history[historyCount - 1].measuredUtc : 0;
  if (lastWindowAt > lastSaved) saveHistory(utc);
  else logMsg("history: no new reading since the last save; nothing saved");
}

static void printHistory() {
  Serial.printf("--- moisture history: saved at midnight and noon (%s), last %d days ---\n",
                LOCAL_TZ, HISTORY_DAYS);
  if (historyCount == 0) Serial.println("(none yet)");
  for (int i = 0; i < historyCount; i++) {
    Serial.printf("%s  %3d%%  (measured %s)\n", fmtLocal(history[i].savedUtc).c_str(), history[i].avg,
                  fmtLocal(history[i].measuredUtc).c_str());
  }
  Serial.println("--------------");
}

// ---- Hourly averages and watering events -----------------------------------

static const int64_t HOURLY_KEEP_S = (int64_t)HOURLY_DAYS * 86400;
static HourlyEntry hourly[HOURLY_MAX];
static int hourlyCount = 0;
static int64_t hourBeingAveraged = -1;  // UTC hour number (utc / 3600)
static int32_t hourSum = 0;
static int hourSamples = 0;
static WateringEvent events[EVENTS_MAX];
static int eventCount = 0;

static void loadHourlyAndEvents() {
  hourlyCount = loadRecords("hourly", hourly, sizeof(HourlyEntry), HOURLY_MAX);
  eventCount = loadRecords("events", events, sizeof(WateringEvent), EVENTS_MAX);
  logMsg("flash: %d hourly averages and %d watering events restored", hourlyCount, eventCount);
}

static void flushHour() {
  if (hourSamples == 0) return;
  HourlyEntry e = {(uint32_t)(hourBeingAveraged * 3600), (int8_t)((hourSum + hourSamples / 2) / hourSamples)};
  hourSum = 0;
  hourSamples = 0;
  appendRecord(hourly, hourlyCount, sizeof(HourlyEntry), HOURLY_MAX, &e, nowUtc() - HOURLY_KEEP_S);
  saveRecords("hourly", hourly, hourlyCount, sizeof(HourlyEntry));
}

static void addHourlySample(int avg) {
  int64_t hour = nowUtc() / 3600;
  if (hourSamples && hour != hourBeingAveraged) flushHour();
  hourBeingAveraged = hour;
  hourSum += avg;
  hourSamples++;
}

static void hourlyTick() {
  if (hourSamples && clockSynced && nowUtc() / 3600 != hourBeingAveraged) flushHour();
}

static void recordWateringEvent(bool completed) {
  if (!clockSynced) return;
  WateringEvent e = {(uint32_t)nowUtc(), (uint8_t)completed, (uint8_t)wateringReason};
  appendRecord(events, eventCount, sizeof(WateringEvent), EVENTS_MAX, &e, 0);
  saveRecords("events", events, eventCount, sizeof(WateringEvent));
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

static void addToWindow(const MoisturePacket &pkt) {
  if (windowPackets == 0) {
    windowStartedAt = uptimeS();
    memset(windowSum, 0, sizeof(windowSum));
    memset(windowValid, 0, sizeof(windowValid));
  }
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (rawIsValid(pkt.raw[i])) {
      windowSum[i] += pkt.raw[i];
      windowValid[i]++;
    }
  }
  windowLastPacket = pkt;
  windowPackets++;
}

// Per-sensor mean of the window's valid readings. A sensor with none keeps
// its last (invalid) raw value, so it still reads as FAULT.
static MoisturePacket windowAverage() {
  MoisturePacket avg = windowLastPacket;
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (windowValid[i]) avg.raw[i] = (uint16_t)((windowSum[i] + windowValid[i] / 2) / windowValid[i]);
  }
  return avg;
}

static void discardWindow() {
  windowPackets = 0;
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

  addToWindow(pkt);
}

static void closeWindow() {
  MoisturePacket pkt = windowAverage();
  int avg = averageMoisture(pkt);
  uint32_t packets = windowPackets;
  uint32_t span = (uint32_t)(uptimeS() - windowStartedAt);
  discardWindow();

  char avgStr[12];
  if (avg >= 0) snprintf(avgStr, sizeof(avgStr), "%d%%", avg);
  else snprintf(avgStr, sizeof(avgStr), "n/a");
  logMsg("DECISION over %lu readings in %s: avg=%s%s", (unsigned long)packets,
       fmtDuration(span).c_str(), avgStr, sensorsToString(pkt, false).c_str());
  haveWindow = true;
  lastWindowPacket = pkt;
  lastWindowAvgAny = avg;
  lastWindowClosedAt = uptimeS();
  if (avg >= 0 && clockSynced) {
    lastWindowAvg = avg;
    lastWindowAt = nowUtc();
    addHourlySample(avg);
  }

  if (watering) {
    logMsg("  -> already watering");
    return;
  }

  uint32_t since = sinceWaterS();
  if (avg < 0) {
    logMsg("  -> no usable sensor, no decision (forced watering in %.1f h still applies)",
         asHours(MAX_GAP_S - min(since, MAX_GAP_S)));
  } else if (avg >= DRY_THRESHOLD_PCT) {
    if (pending == PENDING_DRY) {
      pending = PENDING_NONE;
      logMsg("  -> moist (%d%% >= %d%%); queued dry signal cancelled", avg, DRY_THRESHOLD_PCT);
    } else {
      logMsg("  -> moist (%d%% >= %d%%), no watering", avg, DRY_THRESHOLD_PCT);
    }
  } else if (since < MIN_GAP_S) {
    logMsg("  -> dry (%d%% < %d%%) but watered %.1f h ago; allowed again in %.1f h",
         avg, DRY_THRESHOLD_PCT, asHours(since), asHours(MIN_GAP_S - since));
  } else if (pending != PENDING_NONE) {
    logMsg("  -> dry (%d%% < %d%%); already queued (%s) for %s", avg, DRY_THRESHOLD_PCT,
         pendingName(pending), slotDescription().c_str());
  } else {
    logMsg("  -> dry (%d%% < %d%%) and %.1f h since last watering", avg, DRY_THRESHOLD_PCT, asHours(since));
    setPending(PENDING_DRY);
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
  String queued = pending == PENDING_NONE ? String("none") : String(pendingName(pending)) + " @ " + slotDescription();
  logMsg("HEARTBEAT relay=%s | queued %s | since water %.1f h, dry-water %s, forced in %.1f h | last reading %s | rx %lu missed %lu dup %lu dropped %lu%s%s",
       relay.c_str(), queued.c_str(), asHours(since), allowed.c_str(), asHours(MAX_GAP_S - min(since, MAX_GAP_S)),
       reading.c_str(), (unsigned long)packetsReceived, (unsigned long)packetsMissed,
       (unsigned long)packetsDuplicate, (unsigned long)(droppedWrongSize + droppedWrongSender),
       radioOk ? "" : " | RADIO DOWN", clockSynced ? "" : " | CLOCK NOT SET");
  if (wifiUp) logMsg("HEARTBEAT wifi ch %d %d dBm | web requests %lu", WiFi.channel(), WiFi.RSSI(), (unsigned long)webRequestCount());
  else if (!routerOnEspNowChannel) logMsg("HEARTBEAT WIFI OFF: router not on channel %d, web page down", ESPNOW_CHANNEL);
  else logMsg("HEARTBEAT WIFI DOWN (web page down; retrying)");
  if (pending != PENDING_NONE && !clockSynced) {
    logMsg("WARNING: a watering is queued but the clock isn't set, so it can't be played back at a check");
  }

  uint32_t silentFor = haveReading ? (uint32_t)(uptimeS() - lastReadingAt) : (uint32_t)uptimeS();
  if (silentFor >= NO_READING_WARN_S) {
    logMsg("WARNING: no reading for %.1f h; check the sender (lights, power, range, MAC)", asHours(silentFor));
  }
}

static void printStatus() {
  uint32_t since = sinceWaterS();
  Serial.println("--- status ---");
  uint8_t channel = 0;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&channel, &second);
  Serial.printf("uptime: %s   MAC: %s   channel: %d   radio: %s\n",
                fmtDuration((uint32_t)uptimeS()).c_str(), WiFi.macAddress().c_str(), channel,
                radioOk ? "OK" : "FAILED");
  if (wifiUp) {
    Serial.printf("wifi: connected, IP %s, %d dBm   web: http://%s/ or http://%s.local/ (%lu requests)\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI(), WiFi.localIP().toString().c_str(), MDNS_NAME,
                  (unsigned long)webRequestCount());
  } else if (!routerOnEspNowChannel) {
    Serial.printf("wifi: off - \"%s\" isn't on channel %d (the sender's), so no web page; set the router to channel %d\n",
                  WIFI_SSID, ESPNOW_CHANNEL, ESPNOW_CHANNEL);
  } else {
    Serial.println("wifi: not connected (retrying)");
  }
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
  if (clockSynced) {
    Serial.printf("last watered: %s   total waterings: %lu\n",
                  fmtUtc(nowUtc() - since, true).c_str(), (unsigned long)wateringCount);
    Serial.printf("clock: %s (from %s, synced %s ago%s)\n", fmtUtc(nowUtc(), true).c_str(), clockSource,
                  fmtDuration((uint32_t)(uptimeS() - lastSyncOkAt)).c_str(), lastSyncOk ? "" : ", last resync failed");
  } else {
    Serial.printf("total waterings: %lu\n", (unsigned long)wateringCount);
    Serial.printf("clock: NOT SET (%s)\n", syncAttempted ? "last sync failed" : "not tried yet");
  }
  Serial.printf("queued: %s   next slot: %s\n", pendingName(pending), slotDescription().c_str());
  Serial.printf("history: %d of %d midnight/noon entries (l to list), %d of %d hourly averages, %d watering events\n",
                historyCount, HISTORY_MAX, hourlyCount, HOURLY_MAX, eventCount);
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
  logMsg("settings: controller checks every %d h from 00:00 %s; relay DRY %d min from %d s before | dry below %d%% (averaged over %d s) | min gap %d h | forced after %d days",
       CHECK_EVERY_H, LOCAL_TZ, WATER_DURATION_MIN, WATER_LEAD_S, DRY_THRESHOLD_PCT,
       DECISION_WINDOW_S, MIN_HOURS_BETWEEN_WATERING, MAX_DAYS_WITHOUT_WATER);
  logMsg("settings: Wi-Fi \"%s\" | web port %d, http://%s.local/ | OTA %s | clock from %s (fallback %s), resync every %d h",
       WIFI_SSID, WEB_PORT, MDNS_NAME,
#ifdef OTA_PASSWORD
       "on",
#else
       "off (no OTA_PASSWORD in secrets.h)",
#endif
       TIME_API_URL, NTP_SERVER, CLOCK_RESYNC_H);
  logMsg("settings: moisture history at midnight and noon (%s), kept %d days (%d entries)",
       LOCAL_TZ, HISTORY_DAYS, HISTORY_MAX);
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
  Serial.println("commands: s=status  w=water now  p=queue for next slot  x=stop or cancel watering  c=reset counter  "
                 "f=force fallback  t=sync clock  l=history  n=Wi-Fi scan  r=relay test  d=hold dry  h=help");
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

// Holds the relay DRY with no time cap until any key; the schedule pauses meanwhile.
static void holdDry() {
  if (watering) {
    logMsg("hold dry refused: watering in progress (x to stop it first)");
    return;
  }
  logMsg("HOLD DRY: relay held DRY with no time cap; send any key to release");
  while (Serial.available()) Serial.read();
  setRelayDry(true);
  int64_t startedAt = uptimeS();
  while (!Serial.available()) {
    esp_task_wdt_reset();
    delay(10);
  }
  while (Serial.available()) Serial.read();
  setRelayDry(false);
  logMsg("HOLD DRY released after %s; relay off (controller reads WET)",
       fmtDuration((uint32_t)(uptimeS() - startedAt)).c_str());
}

#if SIMULATE_SENDER
static uint32_t simSeq = 0;
static uint32_t simDryness = 0;
static unsigned long lastSimMs = 0;
static bool simSeeded = false;

static void simulateSender() {
  if (millis() - lastSimMs < 1000) return;
  lastSimMs = millis();
  if (watering) simDryness = 0;
  else simDryness++;
  MoisturePacket pkt;
  pkt.seq = simSeq++;
  for (int i = 0; i < MAX_SENSORS; i++) pkt.raw[i] = (uint16_t)(1850 + i * 30 + simDryness / 10 + random(-6, 7));
  handlePacket(pkt, -40);
}

static void seedSimulatedHistory() {
  if (simSeeded || !clockSynced) return;
  simSeeded = true;
  if (hourlyCount > 0) return;
  int64_t firstHour = nowUtc() / 3600 - HOURLY_MAX;
  float v = 62;
  for (int64_t h = firstHour; h < nowUtc() / 3600; h++) {
    v -= 0.25f;
    if ((h - firstHour) % 150 == 75) v += 22;
    if (localTm(h * 3600).tm_hour < 6) continue;
    HourlyEntry e = {(uint32_t)(h * 3600), (int8_t)v};
    appendRecord(hourly, hourlyCount, sizeof(HourlyEntry), HOURLY_MAX, &e, 0);
  }
  saveRecords("hourly", hourly, hourlyCount, sizeof(HourlyEntry));
  logMsg("SIMULATE: seeded %d hourly averages", hourlyCount);
}
#endif

// Full scan of every channel (ESP-NOW packets are missed for a few seconds).
static void listWifiNetworks() {
  logMsg("scan: all channels for \"%s\"...", WIFI_SSID);
  int16_t n = WiFi.scanNetworks(false, true);
  int matches = 0;
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) != WIFI_SSID) continue;
    matches++;
    logMsg("scan:   channel %2d  %d dBm  access point %s", WiFi.channel(i), WiFi.RSSI(i), WiFi.BSSIDstr(i).c_str());
  }
  logMsg("scan: %d access point(s) with that name, %d networks in total", matches, n < 0 ? 0 : n);
  WiFi.scanDelete();
  restoreEspNowChannel();
}

static void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'r': relayTest(); break;
      case 'd': holdDry(); break;
      case 's': printStatus(); break;
      case 'w': logMsg("command: water now"); startWatering(PENDING_MANUAL); break;
      case 'p': logMsg("command: queue a watering for the next slot"); setPending(PENDING_MANUAL); break;
      case 't': logMsg("command: sync clock"); syncClock(); break;
      case 'x':
        if (!watering && pending != PENDING_NONE) {
          logMsg("command: queued watering (%s) cancelled", pendingName(pending));
          pending = PENDING_NONE;
        } else {
          logMsg("command: stop watering");
          stopWatering(false);
        }
        break;
      case 'c':
        lastWaterAt = uptimeS();
        saveCounter(false, true);
        logMsg("command: counter reset to 0 (as if just watered)");
        break;
      case 'f':
        lastWaterAt = uptimeS() - (int64_t)MAX_GAP_S;
        saveCounter(false, true);
        logMsg("command: counter set to the fallback limit; forced watering next");
        break;
      case 'l': printHistory(); break;
      case 'n': listWifiNetworks(); break;
      case 'h': case '?': printHelp(); break;
      default: break;
    }
  }
}

// ---- Network ---------------------------------------------------------------

// Joins only if the router is on ESPNOW_CHANNEL; the scan covers that channel
// alone, so the radio never leaves the sender's channel.
static void tryJoinOnEspNowChannel() {
  lastWifiCheckAt = uptimeS();
  int16_t found = WiFi.scanNetworks(false, false, false, 300, ESPNOW_CHANNEL, WIFI_SSID);
  uint8_t bssid[6];
  if (found > 0) memcpy(bssid, WiFi.BSSID((uint8_t)0), 6);
  WiFi.scanDelete();
  restoreEspNowChannel();
  if ((found > 0) != routerOnEspNowChannel) {
    routerOnEspNowChannel = found > 0;
    if (!routerOnEspNowChannel) {
      logMsg("WARNING: Wi-Fi \"%s\" not found on channel %d (the sender's); web page off until the router uses "
           "channel %d. Watering is unaffected.", WIFI_SSID, ESPNOW_CHANNEL, ESPNOW_CHANNEL);
    }
  }
  if (found <= 0) return;
  logMsg("wifi: \"%s\" is on channel %d; joining", WIFI_SSID, ESPNOW_CHANNEL);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD, ESPNOW_CHANNEL, bssid);
  wifiJoining = true;
  wifiJoinStartedAt = uptimeS();
}

static void startNetworkServices() {
#ifdef OTA_PASSWORD
  ArduinoOTA.setHostname(MDNS_NAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    logMsg("OTA: firmware upload starting; relay off until the new firmware boots");
    if (watering) stopWatering(false);
    setRelayDry(false);
  });
  ArduinoOTA.onProgress([](unsigned int, unsigned int) { esp_task_wdt_reset(); });
  ArduinoOTA.onEnd([]() { logMsg("OTA: upload done; restarting"); });
  ArduinoOTA.onError([](ota_error_t err) { logMsg("OTA: upload failed (%d)", (int)err); });
  ArduinoOTA.begin();
#else
  MDNS.begin(MDNS_NAME);
#endif
  MDNS.addService("http", "tcp", WEB_PORT);
}

static void wifiTick() {
  bool connected = WiFi.status() == WL_CONNECTED;
  if (connected && WiFi.channel() != ESPNOW_CHANNEL) {
    logMsg("WARNING: router moved to channel %d; leaving Wi-Fi to stay on the sender's channel %d",
         WiFi.channel(), ESPNOW_CHANNEL);
    WiFi.disconnect();
    restoreEspNowChannel();
    connected = false;
  }
  if (connected) {
    wifiJoining = false;
  } else if (wifiJoining && uptimeS() - wifiJoinStartedAt >= WIFI_JOIN_TIMEOUT_S) {
    logMsg("wifi: join timed out (status %d)", (int)WiFi.status());
    WiFi.disconnect();
    restoreEspNowChannel();
    wifiJoining = false;
  }

  if (connected != wifiUp) {
    wifiUp = connected;
    if (connected) {
      esp_wifi_set_ps(WIFI_PS_NONE);
      logMsg("wifi: connected to \"%s\", IP %s, channel %d, %d dBm; web page http://%s/",
           WIFI_SSID, WiFi.localIP().toString().c_str(), WiFi.channel(), WiFi.RSSI(),
           WiFi.localIP().toString().c_str());
      if (!networkServicesStarted) {
        startNetworkServices();
        networkServicesStarted = true;
      }
    } else {
      logMsg("wifi: disconnected; checking again every %d s (watering carries on)", WIFI_RETRY_S);
      restoreEspNowChannel();
    }
  }
  if (!connected && !wifiJoining && uptimeS() - lastWifiCheckAt >= WIFI_RETRY_S) tryJoinOnEspNowChannel();
#ifdef OTA_PASSWORD
  if (connected && networkServicesStarted) ArduinoOTA.handle();
#endif
}

static WebSnapshot webSnap;
static unsigned long lastPublishMs = 0;

static void publishWeb() {
  WebSnapshot &w = webSnap;
  int64_t now = uptimeS();
  uint32_t since = sinceWaterS();
  w.uptimeS = (uint32_t)now;
  w.clockSynced = clockSynced;
  w.nowUtc = clockSynced ? nowUtc() : 0;
  w.wifiRssi = wifiUp ? WiFi.RSSI() : 0;

  w.haveReading = haveWindow;
  w.readingUtc = haveWindow && clockSynced ? nowUtc() - (now - lastWindowClosedAt) : 0;
  w.avg = (int8_t)lastWindowAvgAny;
  for (int i = 0; i < MAX_SENSORS; i++) {
    uint16_t raw = lastWindowPacket.raw[i];
    w.sensors[i] = {SENSORS[i].fitted, rawIsValid(raw), (int8_t)rawToPercent(i, raw), raw};
  }
  w.senderRssi = lastRssi;
  w.lastPacketAgeS = haveReading ? (uint32_t)(now - lastReadingAt) : 0;
  w.packetsReceived = packetsReceived;
  w.packetsMissed = packetsMissed;

  w.watering = watering;
  int64_t left = (int64_t)WATER_S - (now - wateringStartedAt);
  w.wateringLeftS = watering && left > 0 ? (uint32_t)left : 0;
  w.pending = pending;
  w.nextSlotUtc = clockSynced && pending != PENDING_NONE ? nextSlotUtc() : 0;
  w.lastWateredUtc = clockSynced && wateringCount > 0 ? nowUtc() - since : 0;
  w.wateringCount = wateringCount;
  w.dryAllowedInS = MIN_GAP_S - min(since, MIN_GAP_S);
  w.forcedInS = MAX_GAP_S - min(since, MAX_GAP_S);

  w.hourlyCount = hourlyCount;
  memcpy(w.hourly, hourly, hourlyCount * sizeof(HourlyEntry));
  w.historyCount = historyCount;
  memcpy(w.history, history, historyCount * sizeof(HistoryEntry));
  w.eventCount = eventCount;
  memcpy(w.events, events, eventCount * sizeof(WateringEvent));
  webPublish(w);
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
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.setSleep(false);
  WiFi.setHostname(MDNS_NAME);
  WiFi.disconnect();
  restoreEspNowChannel();

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
    logMsg("flash: no saved counter (first boot); starting at %.1f h so a dry reading can queue",
         asHours(since));
  }
  restoredLastWaterUtc = prefs.getLong64("water_utc", 0);
  if (restoredLastWaterUtc > 0) {
    logMsg("flash: last watering was %s; applied once the clock is set", fmtUtc(restoredLastWaterUtc, true).c_str());
  }
  prefs.remove("attempt_hr");
  lastAttemptCheck = prefs.getLong64("attempt_chk", -1);
  applyLocalTz();
  loadHistory();
  loadHourlyAndEvents();
  wateringCount = prefs.getULong("count", 0);
  logMsg("flash: %lu waterings so far", (unsigned long)wateringCount);
  if (prefs.getBool("session", false)) {
    logMsg("WARNING: a watering was cut short by a reset; not counted, not repeated this hour");
    prefs.putBool("session", false);
  }
  lastWaterAt = uptimeS() - (int64_t)since;
  saveCounter(false, false);

  esp_timer_create_args_t timerArgs = {};
  timerArgs.callback = onCutoffTimer;
  timerArgs.name = "water_cutoff";
  if (esp_timer_create(&timerArgs, &cutoffTimer) != ESP_OK) {
    cutoffTimer = nullptr;
    logMsg("WARNING: safety cutoff timer unavailable; loop() still enforces the cap");
  }

  setupWatchdog();

  radioOk = setupRadio();
  webBegin();
  if (!radioOk) {
    // Keep running: the fallback timer still waters without the radio.
    logMsg("running on the fallback timer only");
  }

  static const uint8_t ANY[6] = {0, 0, 0, 0, 0, 0};
  filterSender = memcmp(SENDER_MAC, ANY, 6) != 0;
  if (filterSender) logMsg("radio: accepting packets only from %s", macToString(SENDER_MAC).c_str());
  else logMsg("radio: SENDER_MAC not set; accepting packets from any sender");

#if SIMULATE_SENDER
  logMsg("WARNING: SIMULATE_SENDER is on; readings are generated, not received");
#endif
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

  if (windowPackets && uptimeS() - windowStartedAt >= DECISION_WINDOW_S) closeWindow();

  if (ledBlinkUntilMs && (long)(millis() - ledBlinkUntilMs) >= 0) {
    ledBlinkUntilMs = 0;
    setLed(watering);
  }

  int64_t now = uptimeS();

  if (watering) {
    if (cutoffFired) {
      logMsg("SAFETY CUTOFF: hardware timer ended the session at the %d min cap", MAX_WATERING_SESSION_S / 60);
      stopWatering(true);
    } else if (now - wateringStartedAt >= (int64_t)min(WATER_S, (uint32_t)MAX_WATERING_SESSION_S)) {
      stopWatering(true);
    } else if (now - lastProgressAt >= WATERING_PROGRESS_S) {
      lastProgressAt = now;
      logMsg("watering: %s left", fmtDuration(WATER_S - (uint32_t)(now - wateringStartedAt)).c_str());
    }
  }

  if (!watering && pending != PENDING_FORCED && sinceWaterS() >= MAX_GAP_S) {
    logMsg("fallback: %d days without watering", MAX_DAYS_WITHOUT_WATER);
    setPending(PENDING_FORCED);
  }
  runSlot();
  historyTick();
  hourlyTick();
  wifiTick();
  maybeSyncClock();
  if (millis() - lastPublishMs >= 1000) {
    lastPublishMs = millis();
    publishWeb();
  }

  if (now - lastSaveAt >= (int64_t)SAVE_EVERY_S) {
    saveCounter(true, false);
  }

  if (now - lastHeartbeatAt >= HEARTBEAT_S) {
    lastHeartbeatAt = now;
    heartbeat();
  }

#if SIMULATE_SENDER
  simulateSender();
  seedSimulatedHistory();
#endif
  handleSerial();
  delay(10);
}
