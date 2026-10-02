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
//   - The controller only checks the probe on the hour, so watering is queued
//     and played back at the next X:59:30 (WATER_LEAD_S before the hour, in
//     UTC from world time): relay DRY for WATER_DURATION_MIN, then off. It
//     counts as a watering only once that hold completes.
//   - Average < DRY_THRESHOLD_PCT and at least MIN_HOURS_BETWEEN_WATERING
//     since the last watering -> queue. A later moist window cancels it.
//   - MAX_DAYS_WITHOUT_WATER since the last watering -> queue, reading or not,
//     retried every hour until one completes.
//   - One attempt per hour. A failed attempt (stopped, reset) discards the
//     queued signal and the current window; only new readings can queue again.
//   - The last watering is saved to flash as UTC, so time powered off counts.
//     Before the clock is set, a since-watering counter saved hourly is used.
//   - The clock comes from TIME_API_URL (NTP_SERVER as fallback), joining
//     Wi-Fi for a few seconds every CLOCK_RESYNC_H.
//
// Safety (no endless watering):
//   - Relay off (reads WET) first thing on every boot; losing power closes
//     the NC contact, which also reads WET.
//   - Each session is hard-capped at 7 min (MAX_WATERING_SESSION_S), by loop()
//     and by an independent timer that forces the relay off.
//   - The hour of each attempt is saved to flash before the relay turns on, so
//     a reset mid-session can't repeat it; the watchdog resets the board if
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
// Serial commands (115200 baud): s = status, w = water now, x = stop watering,
// c = reset counter (as if just watered), f = jump counter to the fallback
// limit, p = queue a watering for the next slot, t = sync clock now, r = relay
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
#include <time.h>
#include <stdarg.h>
#include "config.h"
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
static const int64_t SLOT_OFFSET_S = 3600 - WATER_LEAD_S;
static_assert(WATER_LEAD_S > 0 && WATER_LEAD_S < WATER_DURATION_MIN * 60,
              "the DRY hold must start before the hour and last past it");
static_assert(WIFI_CONNECT_TIMEOUT_S < WATCHDOG_TIMEOUT_S, "Wi-Fi join would trip the watchdog");

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

enum PendingReason { PENDING_NONE, PENDING_DRY, PENDING_FORCED, PENDING_MANUAL };
static PendingReason pending = PENDING_NONE;
// UTC hour (utc / 3600) whose X:59:30 slot was last used; -1 = none.
static int64_t lastAttemptHour = -1;

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

// Readings collected toward the next watering decision.
static int64_t windowStartedAt = 0;
static uint32_t windowPackets = 0;
static uint32_t windowSum[MAX_SENSORS];
static uint32_t windowValid[MAX_SENSORS];
static MoisturePacket windowLastPacket;

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

static void startWatering(const char *reason) {
  if (watering) {
    logMsg("already watering; '%s' ignored", reason);
    return;
  }
  logMsg("WATERING START (%s): %.1f h since last watering, relay DRY for %lu min (hard cap %d min)",
       reason, asHours(sinceWaterS()), (unsigned long)WATER_DURATION_MIN, MAX_WATERING_SESSION_S / 60);

  watering = true;
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

// Blocks for up to ~WIFI_CONNECT_TIMEOUT_S + 25 s, feeding the watchdog; ESP-NOW
// packets sent meanwhile are lost.
static void syncClock() {
  syncAttempted = true;
  lastSyncAttemptAt = uptimeS();
  logMsg("clock: joining Wi-Fi \"%s\" to fetch the time", WIFI_SSID);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_S * 1000UL) {
    esp_task_wdt_reset();
    delay(50);
  }

  int64_t utc = 0;
  const char *source = nullptr;
  if (WiFi.status() == WL_CONNECTED) {
    logMsg("clock: Wi-Fi up after %lu ms (channel %d, %d dBm)",
         millis() - start, WiFi.channel(), WiFi.RSSI());
    esp_task_wdt_reset();
    if (fetchTimeApi(utc)) source = "timeapi.io";
    else if (fetchNtp(utc)) source = "NTP";
  } else {
    logMsg("clock: could not join Wi-Fi within %d s (status %d)", WIFI_CONNECT_TIMEOUT_S, (int)WiFi.status());
  }

  WiFi.disconnect();
  for (int i = 0; i < 50 && WiFi.status() == WL_CONNECTED; i++) delay(10);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  uint8_t primary = 0;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&primary, &second);
  if (primary != ESPNOW_CHANNEL) logMsg("WARNING: radio on channel %d after sync, wanted %d", primary, ESPNOW_CHANNEL);

  lastSyncOk = source != nullptr;
  if (lastSyncOk) applyClock(utc, source);
  else logMsg("clock: sync failed; retrying in %d min%s", CLOCK_RETRY_MIN,
              clockSynced ? " (keeping the current clock)" : "");
}

static bool nearSlot() {
  return clockSynced && nowUtc() % 3600 >= SLOT_OFFSET_S - 120;
}

static void maybeSyncClock() {
  if (watering || nearSlot()) return;
  if (syncAttempted) {
    int64_t wait = lastSyncOk ? (int64_t)CLOCK_RESYNC_H * 3600 : (int64_t)CLOCK_RETRY_MIN * 60;
    if (uptimeS() - lastSyncAttemptAt < wait) return;
  }
  syncClock();
}

// ---- Hourly slot -----------------------------------------------------------

static const char *pendingName(PendingReason r) {
  switch (r) {
    case PENDING_DRY: return "soil dry";
    case PENDING_FORCED: return "fallback: max days without water";
    case PENDING_MANUAL: return "manual";
    default: return "none";
  }
}

// UTC start of the next slot (X:59:30) that can still be used.
static int64_t nextSlotUtc() {
  int64_t utc = nowUtc();
  int64_t slot = utc - utc % 3600 + SLOT_OFFSET_S;
  if (utc / 3600 == lastAttemptHour) slot += 3600;
  return slot;
}

static String slotDescription() {
  if (!clockSynced) return "waiting for the clock";
  int64_t slot = nextSlotUtc();
  int64_t wait = slot - nowUtc();
  return fmtUtc(slot, false) + (wait > 0 ? ", in " + fmtDuration((uint32_t)wait) : String(", now"));
}

static void setPending(PendingReason r) {
  if (pending == r) return;
  pending = r;
  if (r != PENDING_NONE) logMsg("  -> QUEUED (%s) for the next slot: %s", pendingName(r), slotDescription().c_str());
}

static void runSlot() {
  if (pending == PENDING_NONE || watering || !clockSynced) return;
  int64_t utc = nowUtc();
  if (utc % 3600 < SLOT_OFFSET_S || utc / 3600 == lastAttemptHour) return;

  lastAttemptHour = utc / 3600;
  prefs.putLong64("attempt_hr", lastAttemptHour);
  PendingReason reason = pending;
  pending = PENDING_NONE;
  if (reason == PENDING_DRY && sinceWaterS() < MIN_GAP_S) {
    logMsg("slot: queued dry signal dropped; watered %.1f h ago", asHours(sinceWaterS()));
    return;
  }
  logMsg("slot: playing back the queued watering for the %s check",
       fmtUtc(utc - utc % 3600 + 3600, false).c_str());
  startWatering(pendingName(reason));
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
  if (pending != PENDING_NONE && !clockSynced) {
    logMsg("WARNING: a watering is queued but the clock isn't set, so it can't be played back on the hour");
  }

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
  logMsg("settings: water %d min from X:%02d:%02d UTC | dry below %d%% (averaged over %d s) | min gap %d h | forced after %d days",
       WATER_DURATION_MIN, (int)(SLOT_OFFSET_S / 60), (int)(SLOT_OFFSET_S % 60), DRY_THRESHOLD_PCT,
       DECISION_WINDOW_S, MIN_HOURS_BETWEEN_WATERING, MAX_DAYS_WITHOUT_WATER);
  logMsg("settings: clock from %s (fallback %s) via Wi-Fi \"%s\", resync every %d h",
       TIME_API_URL, NTP_SERVER, WIFI_SSID, CLOCK_RESYNC_H);
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
  Serial.println("commands: s=status  w=water now  p=queue for next slot  x=stop watering  c=reset counter  "
                 "f=force fallback  t=sync clock  r=relay test  d=hold dry  h=help");
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

static void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'r': relayTest(); break;
      case 'd': holdDry(); break;
      case 's': printStatus(); break;
      case 'w': logMsg("command: water now"); startWatering("manual"); break;
      case 'p': logMsg("command: queue a watering for the next slot"); setPending(PENDING_MANUAL); break;
      case 't': logMsg("command: sync clock"); syncClock(); break;
      case 'x': logMsg("command: stop watering"); stopWatering(false); break;
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
    logMsg("flash: no saved counter (first boot); starting at %.1f h so a dry reading can queue",
         asHours(since));
  }
  restoredLastWaterUtc = prefs.getLong64("water_utc", 0);
  if (restoredLastWaterUtc > 0) {
    logMsg("flash: last watering was %s; applied once the clock is set", fmtUtc(restoredLastWaterUtc, true).c_str());
  }
  lastAttemptHour = prefs.getLong64("attempt_hr", -1);
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
  maybeSyncClock();

  if (now - lastSaveAt >= (int64_t)SAVE_EVERY_S) {
    saveCounter(true, false);
  }

  if (now - lastHeartbeatAt >= HEARTBEAT_S) {
    lastHeartbeatAt = now;
    heartbeat();
  }

  handleSerial();
  delay(10);
}
