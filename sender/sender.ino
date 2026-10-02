// Wireless soil moisture bridge — SENDER
//
// Board: Seeed XIAO ESP32-C6 (Arduino core "esp32" by Espressif, 3.x)
// Tools > USB CDC On Boot: Enabled (for serial output)
//
// Boots when the grow lights come on, reads the raw ADC on A0-A2, and sends
// all three in one ESP-NOW packet to the receiver every SEND_INTERVAL_MS until
// power goes away. No calibration here: the receiver knows which pins have
// sensors and converts raw to moisture, so the sender (hard to reach) never
// needs reflashing for tuning. No sleep or battery logic: the lights are the
// power switch.
//
// Settings live in config.h. Don't connect USB while the buck is powering the
// board; disconnect the 24 V side to flash.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <esp_system.h>
#include "config.h"

// XIAO ESP32-C6 RF switch: GPIO3 low enables it, GPIO14 picks the antenna.
#define RF_SWITCH_ENABLE_PIN 3
#define RF_ANTENNA_SELECT_PIN 14

static volatile bool sendDone = false;
static volatile bool sendOk = false;
static uint32_t seq = 0;
static bool isBroadcast = false;

static unsigned long lastSendMs = 0;

static int ledTogglesLeft = 0;
static unsigned long ledNextToggleMs = 0;

// The send-callback signature changed in ESP-IDF 5.5 (Arduino core 3.3).
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void onSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
#else
static void onSent(const uint8_t *mac, esp_now_send_status_t status) {
#endif
  sendOk = (status == ESP_NOW_SEND_SUCCESS);
  sendDone = true;
}

// Every analog pin is read and sent; the receiver decides which are sensors.
static const uint8_t SENSOR_PINS[MAX_SENSORS] = {A0, A1, A2};

static uint16_t readRaw(int sensor) {
  uint32_t sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(SENSOR_PINS[sensor]);
    delay(ADC_SAMPLE_GAP_MS);
  }
  return (uint16_t)(sum / ADC_SAMPLES);
}

static void readSensors(MoisturePacket &pkt) {
  memset(&pkt, 0, sizeof(pkt));
  for (int i = 0; i < MAX_SENSORS; i++) pkt.raw[i] = readRaw(i);
}

static void printSensors(const MoisturePacket &pkt) {
  for (int i = 0; i < MAX_SENSORS; i++) Serial.printf(" A%d=%u", i, pkt.raw[i]);
}

static void setLed(bool on) {
  digitalWrite(LED_BUILTIN, on ? LOW : HIGH);  // XIAO C6 LED is active-low
}

static void startBlinks(int count) {
  ledTogglesLeft = count * 2 - 1;
  ledNextToggleMs = millis() + SEND_BLINK_ON_MS;
  setLed(true);
}

static void updateLed() {
  if (ledTogglesLeft == 0 || (long)(millis() - ledNextToggleMs) < 0) return;
  bool turnOn = ledTogglesLeft % 2 == 0;
  setLed(turnOn);
  ledTogglesLeft--;
  ledNextToggleMs = millis() + (turnOn ? SEND_BLINK_ON_MS : SEND_BLINK_OFF_MS);
}

static void sendReading() {
  MoisturePacket pkt;
  readSensors(pkt);
  pkt.seq = seq++;

  bool delivered = false;
  int attempt = 0;
  // Broadcast gets no ACK, so the callback reports success after one send.
  while (attempt < SEND_ATTEMPTS && !delivered) {
    attempt++;
    sendDone = false;
    esp_err_t err = esp_now_send(RECEIVER_MAC, (const uint8_t *)&pkt, sizeof(pkt));
    if (err == ESP_OK) {
      unsigned long start = millis();
      while (!sendDone && millis() - start < 200) delay(1);
      delivered = sendDone && sendOk;
    } else {
      Serial.printf("esp_now_send error %d\n", err);
    }
    if (!delivered && attempt < SEND_ATTEMPTS) delay(100);
  }

  Serial.printf("seq=%lu", (unsigned long)pkt.seq);
  printSensors(pkt);
  Serial.printf(" -> %s (attempts: %d)\n",
                delivered ? (isBroadcast ? "broadcast" : "ACKed") : "NOT delivered",
                attempt);

  startBlinks(delivered ? 1 : SEND_FAIL_BLINKS);
}

void setup() {
  Serial.begin(115200);
  unsigned long waitStart = millis();
  while (!Serial && millis() - waitStart < 1000) delay(10);

  pinMode(RF_SWITCH_ENABLE_PIN, OUTPUT);
  digitalWrite(RF_SWITCH_ENABLE_PIN, LOW);
  pinMode(RF_ANTENNA_SELECT_PIN, OUTPUT);
  digitalWrite(RF_ANTENNA_SELECT_PIN, USE_EXTERNAL_ANTENNA ? HIGH : LOW);

  analogReadResolution(12);
  for (int i = 0; i < MAX_SENSORS; i++) pinMode(SENSOR_PINS[i], INPUT);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  pinMode(LED_BUILTIN, OUTPUT);
  setLed(false);

  Serial.println();
  Serial.println("=== Moisture sender ===");
  // Brownout here usually means the buck or dimmer can't hold the supply up.
  Serial.printf("Reset reason: %d (1=power on, 9=brownout)\n", (int)esp_reset_reason());
  Serial.printf("This board's MAC: %s  (put it in receiver/config.h SENDER_MAC)\n",
                WiFi.macAddress().c_str());
  Serial.printf("Channel %d, interval %d ms, sending raw A0-A2 (receiver interprets)\n",
                ESPNOW_CHANNEL, SEND_INTERVAL_MS);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed; restarting in 5 s");
    delay(5000);
    ESP.restart();
  }
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, RECEIVER_MAC, 6);
  peer.channel = 0;  // current channel
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Adding receiver peer failed; restarting in 5 s");
    delay(5000);
    ESP.restart();
  }

  static const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  isBroadcast = memcmp(RECEIVER_MAC, BROADCAST, 6) == 0;
  if (isBroadcast) {
    Serial.println("WARNING: RECEIVER_MAC is broadcast; set the receiver's MAC before installing.");
  }

  delay(SENSOR_WARMUP_MS);
  lastSendMs = millis();
  sendReading();
}

void loop() {
  if (millis() - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = millis();
    sendReading();
  }
  updateLed();
  delay(1);
}
