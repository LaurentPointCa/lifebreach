#ifndef CONFIG_H
#define CONFIG_H

// ── MQTT topics ─────────────────────────────────────────────────────────────

#define MQTT_TOPIC_STATE       "lifebreach/state"
#define MQTT_TOPIC_AVAILABLE   "lifebreach/available"
#define MQTT_TOPIC_COMMAND     "lifebreach/command"

// ── MQTT timing ─────────────────────────────────────────────────────────────

#define MQTT_HEARTBEAT_MS      30000   // publish state every 30s even if unchanged
#define MQTT_RECONNECT_MS      5000    // retry MQTT connection every 5s

// ── WiFiManager ─────────────────────────────────────────────────────────────

#define WIFI_AP_NAME           "LifeBreach-Setup"
#define WIFI_PORTAL_TIMEOUT    180     // seconds before giving up on portal and continuing without WiFi

// ── OTA ─────────────────────────────────────────────────────────────────────

#define OTA_HOSTNAME           "lifebreach"
#define OTA_PASSWORD           "lifebreach"

// ── HA Discovery ────────────────────────────────────────────────────────────

#define HA_DISCOVERY_PREFIX    "homeassistant"
#define HA_DEVICE_ID           "lifebreach_hrv"
#define HA_DEVICE_NAME         "LifeBreach HRV"

#endif // CONFIG_H
