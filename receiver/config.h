#pragma once
#include <stdint.h>

// ============================================================================
// Receiver settings — always-on XIAO ESP32-C6 next to the watering controller.
// ============================================================================

// ---- Schedule --------------------------------------------------------------

// The controller looks at the probe once an hour, on the hour (X:00), so a
// decision to water is queued and played back at the next hour: the relay
// goes DRY WATER_LEAD_S before X:00 and stays DRY for WATER_DURATION_MIN.
// A watering only counts once that hold has run to the end. The hour comes
// from world time (UTC) over Wi-Fi, so the controller's clock must be on a
// whole-hour time zone and roughly right.
#define WATER_LEAD_S 30

// How long the relay signals "dry" (real minutes, not affected by TIME_SCALE).
#define WATER_DURATION_MIN 5

// Water when the average moisture of the fitted sensors is below this.
// Reference points for sensor A0 (2026-09-28, full list in SENSORS below):
//   bag-dry 0% | mostly dry 7% | damp, good watering point 31% (raw ~2080)
//   | just under saturated 86% | saturated 100%
// Measured in rootless soil; real bed soil may read a little differently, so
// check the daily readings after installing and adjust.
#define DRY_THRESHOLD_PCT 35

// Readings are averaged over this many real seconds (per sensor) before each
// watering decision. The sender reports every second.
#define DECISION_WINDOW_S 20

// Queue a watering regardless of moisture after this many days without one.
#define MAX_DAYS_WITHOUT_WATER 10

// Never water more often than this (limits watering to once per day).
#define MIN_HOURS_BETWEEN_WATERING 20

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

// ---- Logging ---------------------------------------------------------------

// One-line "still alive" summary this often (real seconds).
#define HEARTBEAT_S 60

// Progress line this often while watering (real seconds).
#define WATERING_PROGRESS_S 30

// Warn in the heartbeat when no reading has arrived for this long (hours,
// scaled by TIME_SCALE). Lights come on daily, so a gap over a day is suspect.
#define NO_READING_WARN_H 26

// Moisture history (serial 'l'): the first average of the fitted sensors in
// each HISTORY_INTERVAL_H block of UTC (00:00Z, 12:00Z) is saved to flash.
// Blocks with no readings (lights off) have no entry. Entries older than
// HISTORY_DAYS are dropped.
#define HISTORY_INTERVAL_H 12
#define HISTORY_DAYS 14

// Flash the LED briefly on every packet received (the LED stays on while
// watering).
#define BLINK_ON_PACKET 1

// ---- Clock -----------------------------------------------------------------

// Wi-Fi name and password live in secrets.h (copy secrets.example.h). The
// receiver joins only for a few seconds per sync, then returns to
// ESPNOW_CHANNEL; packets sent meanwhile are lost.
#define TIME_API_URL "https://timeapi.io/api/v1/time/current/utc"
#define NTP_SERVER "pool.ntp.org"  // used when TIME_API_URL fails

// Resync this often once the clock is set; retry this often after a failure.
#define CLOCK_RESYNC_H 6
#define CLOCK_RETRY_MIN 5

// Give up joining Wi-Fi after this long. Keep well under WATCHDOG_TIMEOUT_S.
#define WIFI_CONNECT_TIMEOUT_S 15

// ---- Radio -----------------------------------------------------------------

// Wi-Fi channel for ESP-NOW. Must match the sender.
#define ESPNOW_CHANNEL 1

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
