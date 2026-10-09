#pragma once
#include <stdint.h>

// ============================================================================
// Receiver settings — always-on XIAO ESP32-C6 next to the watering controller.
// ============================================================================

// ---- Schedule --------------------------------------------------------------

// The controller looks at the probe every CHECK_EVERY_H hours on the hour,
// counted from midnight in LOCAL_TZ (00:00, 04:00, 08:00, ...), so a decision
// to water is queued and played back at its next check: the relay goes DRY
// WATER_LEAD_S before the check and stays DRY for WATER_DURATION_MIN. A
// watering only counts once that hold has run to the end. The time comes from
// world time over Wi-Fi, so the controller's clock must be roughly right.
#define CHECK_EVERY_H 4
#define WATER_LEAD_S 30

// Local time zone (POSIX TZ string) for the controller's checks and the
// midnight/noon history: US Pacific with daylight saving (PST/PDT).
#define LOCAL_TZ "PST8PDT,M3.2.0,M11.1.0"

// How long the relay signals "dry" (real minutes, not affected by TIME_SCALE).
#define WATER_DURATION_MIN 5

// Watering cycle on the average moisture of the fitted sensors: at or below
// WATER_START_PCT a cycle starts and a watering is queued for every controller
// check until the average reads above WATER_STOP_PCT; then it waits for
// WATER_START_PCT again.
// Reference points for sensor A0 (2026-09-28, full list in SENSORS below):
//   bag-dry 0% | mostly dry 7% | damp, good watering point 31% (raw ~2080)
//   | just under saturated 86% | saturated 100%
#define WATER_START_PCT 35
#define WATER_STOP_PCT 50

// A cycle ends after this many waterings even if the average never passed
// WATER_STOP_PCT, and at most one cycle starts per MIN_HOURS_BETWEEN_CYCLES
// (counted from the previous cycle's start).
#define MAX_CYCLE_WATERINGS 3
#define MIN_HOURS_BETWEEN_CYCLES 24

// Readings are averaged over this many real seconds (per sensor) before each
// watering decision. The sender reports every second.
#define DECISION_WINDOW_S 20

// Queue a watering regardless of moisture after this many days without one.
#define MAX_DAYS_WITHOUT_WATER 10

// Test speed-up: divides the length of an "hour" (and so a "day").
//   1   = normal
//   60  = 1 hour -> 1 min,  1 day -> 24 min,  10 days -> 4 h
//   600 = 1 hour -> 6 s,    1 day -> 2.4 min, 10 days -> 24 min
// The counter saved in flash is in real seconds, so reset it (serial 'c')
// after switching back to 1.
#define TIME_SCALE 1

// ---- Hardware --------------------------------------------------------------

// GPIO driving the relay module's IN (or the Form B SSR's LED).
#define RELAY_PIN D1

// 1 = high-level-trigger relay module / SSR LED driven from the pin.
// 0 = low-level-trigger module.
#define RELAY_ACTIVE_HIGH 1

// On-board user LED lights while watering. The XIAO C6 LED is active-low.
#define STATUS_LED_PIN LED_BUILTIN
#define STATUS_LED_ACTIVE_LOW 1

// 0 = on-board ceramic antenna, 1 = external U.FL antenna.
#define USE_EXTERNAL_ANTENNA 0

// Reset the board if loop() stalls this long; the relay drops out on reboot.
#define WATCHDOG_TIMEOUT_S 30

// Relay test (serial 'r'): seconds in each state, long enough for a meter to settle.
#define RELAY_TEST_S 20

// Bench testing without a sender: 1 = generate a reading every second (slowly
// drying, re-wetted by each completed watering) and fill an empty hourly
// history with 14 days of sample data. Build with -DSIMULATE_SENDER=1 rather
// than changing it here.
#ifndef SIMULATE_SENDER
#define SIMULATE_SENDER 0
#endif

// ---- Logging ---------------------------------------------------------------

// One-line "still alive" summary this often (real seconds).
#define HEARTBEAT_S 60

// Progress line this often while watering (real seconds).
#define WATERING_PROGRESS_S 30

// Warn in the heartbeat when no reading has arrived for this long (hours,
// scaled by TIME_SCALE). Lights come on daily, so a gap over a day is suspect.
#define NO_READING_WARN_H 26

// Moisture history (serial 'l'): at midnight and noon in LOCAL_TZ, the most
// recent average of the fitted sensors is saved to flash with the time it was
// measured (the lights may be off at midnight). Nothing is saved if there has
// been no new reading since the last save. Entries older than HISTORY_DAYS are
// dropped.
#define HISTORY_DAYS 14

// Flash the LED briefly on every packet received (the LED stays on while
// watering).
#define BLINK_ON_PACKET 1

// ---- Clock -----------------------------------------------------------------

#define TIME_API_URL "https://timeapi.io/api/v1/time/current/utc"
#define NTP_SERVER "pool.ntp.org"  // used when TIME_API_URL fails

// Resync this often once the clock is set; retry this often after a failure.
#define CLOCK_RESYNC_H 6
#define CLOCK_RETRY_MIN 5

// ---- Network ---------------------------------------------------------------

// Wi-Fi name and password (and optional OTA_PASSWORD) live in secrets.h; copy
// secrets.example.h. The receiver stays on Wi-Fi only while the router is on
// ESPNOW_CHANNEL; it checks for it every WIFI_RETRY_S and gives up a join
// after WIFI_JOIN_TIMEOUT_S (keep it under WATCHDOG_TIMEOUT_S).
#define WIFI_RETRY_S 60
#define WIFI_JOIN_TIMEOUT_S 15

// Read-only web page and JSON API. On the home network the receiver is also
// http://MDNS_NAME.local/ and accepts OTA uploads under that name.
#define WEB_PORT 80
#define MDNS_NAME "moisture"

// Hourly average moisture kept for the chart (one flash write per hour).
#define HOURLY_DAYS 14

// ---- Radio -----------------------------------------------------------------

// Wi-Fi channel for ESP-NOW. Must match the sender, which is fixed on 1. The
// radio never leaves it except for brief clock syncs, so the web page is only
// up when the router's 2.4 GHz Wi-Fi is set to this channel (not "auto").
// Otherwise watering carries on and the clock is synced by joining Wi-Fi for a
// few seconds at a time, as with no web page.
#ifndef ESPNOW_CHANNEL
#define ESPNOW_CHANNEL 1
#endif

// Accept packets only from this MAC (printed on the sender's serial console).
// All zeros = accept any sender (fine for bench testing).
static const uint8_t SENDER_MAC[6] = {0x10, 0xBD, 0xA3, 0xAF, 0x35, 0x0C};

// ---- Sensors ---------------------------------------------------------------

#define MAX_SENSORS 3

// The sender sends raw ADC (0-4095) from A0, A1 and A2 whether or not a
// sensor is wired there; this table says which are real and how to read them.
// Changing any of it only needs the receiver reflashed.
//   fitted: a sensor is wired to this pin (unfitted pins float; ignored)
//   rawDry: raw reading in dry soil      -> 0 %
//   rawWet: raw reading in saturated soil -> 100 %
// Capacitive sensors read higher when drier. Calibrate with the sensor inserted
// to its final depth; shallower insertion reads much drier.
struct SensorConfig {
  bool fitted;
  uint16_t rawDry;
  uint16_t rawWet;
};

static const SensorConfig SENSORS[MAX_SENSORS] = {
  // A0: measured 2026-09-28 (raw -> % with dry 2460 / wet 1240):
  //   air (out of soil)                        ~2690  ->   0% (clamped)
  //   bag-dry soil, fully inserted             ~2460  ->   0%   rawDry
  //   mostly dry, lightly dampened             ~2370  ->   7%
  //   damp, good watering point, no roots      ~2080  ->  31%
  //   same soil 24 h later (2026-09-29)        ~2122  ->  27%   (~3.5 pts/day, no plants)
  //   just under saturated, fully inserted     ~1410  ->  86%
  //   saturated (water pooling)                ~1242  -> 100%   rawWet
  // Depth matters: the watering-point soil read ~2425 (3%) and wet soil ~2085
  // when only partly inserted. Install at the same full depth.
  // All three compared 2026-10-01:
  //   in air:    A0 ~2695, A1 ~2697, A2 ~2689   (within 8 counts)
  //   in water:  A0 ~1047, A1 ~1060, A2 ~1056   (within 13 counts)
  // The sensors match at both ends, so all three share A0's soil calibration.
  // Water reads wetter than saturated soil (~1242); soil is the scale's 100%.
  // Placement matters far more than the sensor: one sensor read 2499, 2099 and
  // (another) 1827 in the same cup of damp soil depending on depth and how
  // firmly the soil was packed against it.
  {true, 2460, 1240},  // A0
  {true, 2460, 1240},  // A1
  {true, 2460, 1240},  // A2
};

// A fitted sensor reading outside this range is treated as faulty (unplugged,
// shorted, or waterlogged electronics) and left out of the average.
#define RAW_VALID_MIN 300
#define RAW_VALID_MAX 3800

// ---- Packet (must match sender/config.h exactly) ---------------------------

typedef struct __attribute__((packed)) {
  uint32_t seq;               // increments per reading; restarts at 0 each boot
  uint16_t raw[MAX_SENSORS];  // averaged ADC (0-4095) on A0, A1, A2
} MoisturePacket;
