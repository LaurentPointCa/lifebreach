#ifndef WIFI_MQTT_H
#define WIFI_MQTT_H

#include <Adafruit_SSD1306.h>

// ── Connection state (read by display code) ────────────────────────────────

extern bool wifi_connected;
extern bool mqtt_connected;

// ── Public API ─────────────────────────────────────────────────────────────

void wifi_setup(Adafruit_SSD1306& display);
void mqtt_loop();

#endif // WIFI_MQTT_H
