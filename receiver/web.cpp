#include <Arduino.h>
#include <WebServer.h>
#include <time.h>
#include "web.h"
#include "web_page.h"

static WebServer server(WEB_PORT);
static SemaphoreHandle_t snapMutex = nullptr;
static WebSnapshot published;
static WebSnapshot snap;  // web task only
static volatile uint32_t requests = 0;

void webPublish(const WebSnapshot &s) {
  if (!snapMutex) return;
  xSemaphoreTake(snapMutex, portMAX_DELAY);
  published = s;
  xSemaphoreGive(snapMutex);
}

uint32_t webRequestCount() {
  return requests;
}

static void takeSnapshot() {
  xSemaphoreTake(snapMutex, portMAX_DELAY);
  snap = published;
  xSemaphoreGive(snapMutex);
}

static void appendIso(String &out, int64_t utc) {
  if (utc <= 0) {
    out += "null";
    return;
  }
  time_t t = (time_t)utc;
  struct tm tm;
  gmtime_r(&t, &tm);
  char buf[24];
  strftime(buf, sizeof(buf), "\"%Y-%m-%dT%H:%M:%SZ\"", &tm);
  out += buf;
}

static void appendField(String &out, const char *name) {
  if (out.length() && out[out.length() - 1] != '{' && out[out.length() - 1] != '[') out += ',';
  out += '"';
  out += name;
  out += "\":";
}

static void sendJson(const String &body) {
  requests = requests + 1;
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", body);
}

static String currentJson() {
  String j;
  j.reserve(1024);
  j += '{';
  appendField(j, "time"); appendIso(j, snap.clockSynced ? snap.nowUtc : 0);
  appendField(j, "clockSet"); j += snap.clockSynced ? "true" : "false";
  appendField(j, "uptimeS"); j += (unsigned long)snap.uptimeS;
  appendField(j, "thresholdPct"); j += DRY_THRESHOLD_PCT;

  appendField(j, "reading");
  if (snap.haveReading) {
    j += '{';
    appendField(j, "time"); appendIso(j, snap.readingUtc);
    appendField(j, "windowS"); j += DECISION_WINDOW_S;
    appendField(j, "avg"); if (snap.avg >= 0) j += (int)snap.avg; else j += "null";
    appendField(j, "sensors"); j += '[';
    for (int i = 0; i < MAX_SENSORS; i++) {
      const WebSensor &s = snap.sensors[i];
      if (i) j += ',';
      j += "{\"pin\":\"A";
      j += i;
      j += "\",\"fitted\":";
      j += s.fitted ? "true" : "false";
      j += ",\"status\":\"";
      j += !s.fitted ? "unfitted" : s.valid ? "ok" : "fault";
      j += "\",\"pct\":";
      if (s.fitted && s.valid) j += (int)s.pct; else j += "null";
      j += ",\"raw\":";
      j += (int)s.raw;
      j += '}';
    }
    j += "]}";
  } else {
    j += "null";
  }

  appendField(j, "watering");
  j += '{';
  appendField(j, "active"); j += snap.watering ? "true" : "false";
  appendField(j, "remainingS"); j += (unsigned long)(snap.watering ? snap.wateringLeftS : 0);
  appendField(j, "queued"); j += '"'; j += pendingName(snap.pending); j += '"';
  appendField(j, "nextSlot"); appendIso(j, snap.nextSlotUtc);
  appendField(j, "lastWatered"); appendIso(j, snap.lastWateredUtc);
  appendField(j, "count"); j += (unsigned long)snap.wateringCount;
  appendField(j, "dryAllowedInS"); j += (unsigned long)snap.dryAllowedInS;
  appendField(j, "forcedInS"); j += (unsigned long)snap.forcedInS;
  j += '}';

  appendField(j, "radio");
  j += '{';
  appendField(j, "senderRssi"); if (snap.packetsReceived) j += snap.senderRssi; else j += "null";
  appendField(j, "lastPacketAgeS"); if (snap.packetsReceived) j += (unsigned long)snap.lastPacketAgeS; else j += "null";
  appendField(j, "packetsReceived"); j += (unsigned long)snap.packetsReceived;
  appendField(j, "packetsMissed"); j += (unsigned long)snap.packetsMissed;
  appendField(j, "wifiRssi"); j += snap.wifiRssi;
  j += "}}";
  return j;
}

static String historyJson(int days) {
  int64_t since = snap.nowUtc - (int64_t)days * 86400;
  String j;
  j.reserve(64 + snap.hourlyCount * 40 + snap.historyCount * 72 + snap.eventCount * 64);
  j += "{\"days\":";
  j += days;
  j += ",\"hourly\":[";
  bool first = true;
  for (int i = 0; i < snap.hourlyCount; i++) {
    if ((int64_t)snap.hourly[i].utc < since) continue;
    if (!first) j += ',';
    first = false;
    j += "{\"t\":"; appendIso(j, snap.hourly[i].utc);
    j += ",\"avg\":"; j += (int)snap.hourly[i].avg;
    j += '}';
  }
  j += "],\"snapshots\":[";
  first = true;
  for (int i = 0; i < snap.historyCount; i++) {
    if ((int64_t)snap.history[i].savedUtc < since) continue;
    if (!first) j += ',';
    first = false;
    j += "{\"t\":"; appendIso(j, snap.history[i].savedUtc);
    j += ",\"measured\":"; appendIso(j, snap.history[i].measuredUtc);
    j += ",\"avg\":"; j += (int)snap.history[i].avg;
    j += '}';
  }
  j += "],\"waterings\":[";
  first = true;
  for (int i = 0; i < snap.eventCount; i++) {
    if ((int64_t)snap.events[i].utc < since) continue;
    if (!first) j += ',';
    first = false;
    j += "{\"t\":"; appendIso(j, snap.events[i].utc);
    j += ",\"completed\":"; j += snap.events[i].completed ? "true" : "false";
    j += ",\"reason\":\""; j += pendingName((PendingReason)snap.events[i].reason); j += "\"}";
  }
  j += "]}";
  return j;
}

static void handleRoot() {
  requests = requests + 1;
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "text/html; charset=utf-8", WEB_PAGE);
}

static void handleCurrent() {
  takeSnapshot();
  sendJson(currentJson());
}

static void handleHistory() {
  takeSnapshot();
  int days = server.hasArg("days") ? server.arg("days").toInt() : HOURLY_DAYS;
  if (days < 1 || days > HOURLY_DAYS) days = HOURLY_DAYS;
  sendJson(historyJson(days));
}

static void handleNotFound() {
  requests = requests + 1;
  server.send(404, "text/plain", "not found\n");
}

static void webTask(void *) {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/current", HTTP_GET, handleCurrent);
  server.on("/api/history", HTTP_GET, handleHistory);
  server.onNotFound(handleNotFound);
  server.begin();
  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void webBegin() {
  snapMutex = xSemaphoreCreateMutex();
  xTaskCreate(webTask, "web", 8192, nullptr, 1, nullptr);
}
