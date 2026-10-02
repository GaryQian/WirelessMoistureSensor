#pragma once
#include <stdint.h>

// ============================================================================
// Sender settings — XIAO ESP32-C6 by the plants, powered from the LED dimmer.
// ============================================================================

// ---- Radio -----------------------------------------------------------------

// MAC of the receiver board (printed on its serial console at boot).
// Set to broadcast {0xFF x6} to work with any receiver, but then there's no
// delivery ACK, so lost packets can't be retried.
static const uint8_t RECEIVER_MAC[6] = {0x10, 0xBD, 0xA3, 0xB1, 0x40, 0x1C};

// Wi-Fi channel for ESP-NOW. Must match the receiver. If either board ever
// joins Wi-Fi, set this to the router's channel on both.
#define ESPNOW_CHANNEL 1

// 0 = on-board ceramic antenna, 1 = external U.FL antenna.
#define USE_EXTERNAL_ANTENNA 0

// ---- Reporting -------------------------------------------------------------

// How often to repeat the reading while powered (lights on).
#define SEND_INTERVAL_MS 1000

// Delivery attempts per reading when no ACK comes back (unicast only).
#define SEND_ATTEMPTS 3

// Let the sensor's oscillator and the supply settle before the first read.
#define SENSOR_WARMUP_MS 2000

// The on-board LED blinks after every send: once = delivered, SEND_FAIL_BLINKS
// times = failed. The steady blinking doubles as the power indicator.
#define SEND_BLINK_ON_MS 60
#define SEND_BLINK_OFF_MS 120
#define SEND_FAIL_BLINKS 3

// ---- Sensor ----------------------------------------------------------------

// The sender is hard to reach, so it does no interpreting: it always sends the
// raw ADC reading of every analog pin (A0, A1, A2), fitted or not. Which pins
// have a sensor, the calibration and fault limits all live in
// receiver/config.h, so none of them need the sender reflashed.
// All sensors share 3V3 and GND.

#define ADC_SAMPLES 16          // averaged per reading
#define ADC_SAMPLE_GAP_MS 5

// ---- Packet (must match receiver/config.h exactly) -------------------------

#define MAX_SENSORS 3

typedef struct __attribute__((packed)) {
  uint32_t seq;               // increments per reading; restarts at 0 each boot
  uint16_t raw[MAX_SENSORS];  // averaged ADC (0-4095) on A0, A1, A2
} MoisturePacket;
