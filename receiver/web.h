#pragma once
#include <stdint.h>
#include "config.h"

enum PendingReason : uint8_t { PENDING_NONE, PENDING_DRY, PENDING_FORCED, PENDING_MANUAL };

inline const char *pendingName(PendingReason r) {
  switch (r) {
    case PENDING_DRY: return "watering cycle";
    case PENDING_FORCED: return "fallback: max days without water";
    case PENDING_MANUAL: return "manual";
    default: return "none";
  }
}

// Flash record layouts; changing one needs a new Preferences key.
struct __attribute__((packed)) HourlyEntry {
  uint32_t utc;  // start of the UTC hour
  int8_t avg;    // mean of that hour's window averages
};

struct __attribute__((packed)) HistoryEntry {
  uint32_t savedUtc;
  uint32_t measuredUtc;
  int8_t avg;
};

struct __attribute__((packed)) WateringEvent {
  uint32_t utc;  // when the hold ended
  uint8_t completed;
  uint8_t reason;  // PendingReason
};

static const int HOURLY_MAX = HOURLY_DAYS * 24;
static const int HISTORY_MAX = HISTORY_DAYS * 2;
static const int EVENTS_MAX = 32;

struct WebSensor {
  bool fitted;
  bool valid;
  int8_t pct;
  uint16_t raw;
};

// Everything the web pages show, copied from loop() so the server task never
// touches live state. Times are UTC seconds; 0 = unknown.
struct WebSnapshot {
  uint32_t uptimeS;
  int64_t nowUtc;
  bool clockSynced;
  int wifiRssi;

  bool haveReading;
  int64_t readingUtc;
  int8_t avg;
  WebSensor sensors[MAX_SENSORS];
  int senderRssi;
  uint32_t lastPacketAgeS;
  uint32_t packetsReceived;
  uint32_t packetsMissed;

  bool watering;
  uint32_t wateringLeftS;
  PendingReason pending;
  int64_t nextSlotUtc;
  int64_t lastWateredUtc;
  uint32_t wateringCount;
  bool cycleActive;
  uint32_t cycleWaterings;
  int64_t nextCycleAllowedUtc;
  uint32_t forcedInS;

  int hourlyCount;
  HourlyEntry hourly[HOURLY_MAX];
  int historyCount;
  HistoryEntry history[HISTORY_MAX];
  int eventCount;
  WateringEvent events[EVENTS_MAX];
};

// Starts the HTTP server in its own task, so slow clients can't stall loop().
void webBegin();
void webPublish(const WebSnapshot &snap);
uint32_t webRequestCount();
