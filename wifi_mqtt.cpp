#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include "wifi_mqtt.h"
#include "hrv_uart.h"
#include "hrv_protocol.h"
#include "config.h"

// ── Connection state ───────────────────────────────────────────────────────

bool wifi_connected  = false;
bool mqtt_connected  = false;

// ── Private state ──────────────────────────────────────────────────────────

static WiFiClient   espClient;
static PubSubClient mqtt(espClient);
static Preferences  prefs;

static char mqtt_server[40] = "";
static char mqtt_user[20]   = "";
static char mqtt_pass[20]   = "";
static uint32_t last_mqtt_reconnect_ms = 0;
static uint32_t last_mqtt_publish_ms   = 0;
static bool     ha_discovery_sent      = false;

static hrv_state_t prev_mqtt_state = {MODE_UNKNOWN, 0, false, false, 0, 0, 0};

// ── Command options ────────────────────────────────────────────────────────
// Must match HA select discovery options exactly

struct mqtt_cmd_option_t {
    const char* label;
    hrv_mode_t  mode;
    uint8_t     fan;
};

static const mqtt_cmd_option_t MQTT_CMD_OPTIONS[] = {
    {"Off",       MODE_UNKNOWN, 0},
    {"Fresh 1",   MODE_FRESH,   1},
    {"Fresh 2",   MODE_FRESH,   2},
    {"Fresh 3",   MODE_FRESH,   3},
    {"Fresh 4",   MODE_FRESH,   4},
    {"Fresh 5",   MODE_FRESH,   5},
    {"Recirc 1",  MODE_RECIRC,  1},
    {"Recirc 2",  MODE_RECIRC,  2},
    {"Recirc 3",  MODE_RECIRC,  3},
    {"Recirc 4",  MODE_RECIRC,  4},
    {"Recirc 5",  MODE_RECIRC,  5},
};
#define MQTT_CMD_OPTIONS_COUNT (sizeof(MQTT_CMD_OPTIONS) / sizeof(MQTT_CMD_OPTIONS[0]))

static const char* current_cmd_label = "Off";

// ── MQTT callback ──────────────────────────────────────────────────────────

static void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    char buf[32];
    unsigned int len = min(length, (unsigned int)(sizeof(buf) - 1));
    memcpy(buf, payload, len);
    buf[len] = '\0';

    Serial.printf("MQTT cmd: %s\n", buf);

    for (int i = 0; i < MQTT_CMD_OPTIONS_COUNT; i++) {
        if (strcmp(buf, MQTT_CMD_OPTIONS[i].label) == 0) {
            const mqtt_cmd_option_t* opt = &MQTT_CMD_OPTIONS[i];
            if (opt->mode == MODE_UNKNOWN || opt->fan == 0) {
                tx_command_byte = 0xFF;
                tx_active = false;
            } else {
                tx_command_byte = hrv_lcd_command_byte(opt->mode, opt->fan);
                tx_active = true;
            }
            current_cmd_label = opt->label;
            Serial.printf("MQTT -> TX: 0x%02X (%s)\n", tx_command_byte, opt->label);

            mqtt.publish(MQTT_TOPIC_CMD_STATE, opt->label, true);
            return;
        }
    }

    Serial.printf("MQTT: unknown command '%s'\n", buf);
}

// ── HA auto-discovery ──────────────────────────────────────────────────────

static void mqtt_publish_ha_discovery() {
    char topic[128];
    char payload[512];

    const char* dev_json =
        "\"dev\":{\"ids\":[\"" HA_DEVICE_ID "\"],"
        "\"name\":\"" HA_DEVICE_NAME "\","
        "\"mf\":\"LifeBreach\","
        "\"mdl\":\"ESP32 HRV Controller\"}";

    // Sensor: Mode
    snprintf(topic, sizeof(topic),
             "%s/sensor/%s_mode/config", HA_DISCOVERY_PREFIX, HA_DEVICE_ID);
    snprintf(payload, sizeof(payload),
             "{\"name\":\"HRV Mode\","
             "\"stat_t\":\"" MQTT_TOPIC_STATE "\","
             "\"val_tpl\":\"{{value_json.mode}}\","
             "\"uniq_id\":\"%s_mode\","
             "\"avty_t\":\"" MQTT_TOPIC_AVAILABLE "\","
             "%s}", HA_DEVICE_ID, dev_json);
    mqtt.publish(topic, payload, true);

    // Sensor: Fan Speed
    snprintf(topic, sizeof(topic),
             "%s/sensor/%s_fan/config", HA_DISCOVERY_PREFIX, HA_DEVICE_ID);
    snprintf(payload, sizeof(payload),
             "{\"name\":\"HRV Fan Speed\","
             "\"stat_t\":\"" MQTT_TOPIC_STATE "\","
             "\"val_tpl\":\"{{value_json.fan}}\","
             "\"uniq_id\":\"%s_fan\","
             "\"ic\":\"mdi:fan\","
             "\"avty_t\":\"" MQTT_TOPIC_AVAILABLE "\","
             "%s}", HA_DEVICE_ID, dev_json);
    mqtt.publish(topic, payload, true);

    // Sensor: Uptime
    snprintf(topic, sizeof(topic),
             "%s/sensor/%s_uptime/config", HA_DISCOVERY_PREFIX, HA_DEVICE_ID);
    snprintf(payload, sizeof(payload),
             "{\"name\":\"HRV Uptime\","
             "\"stat_t\":\"" MQTT_TOPIC_STATE "\","
             "\"val_tpl\":\"{{value_json.uptime}}\","
             "\"uniq_id\":\"%s_uptime\","
             "\"unit_of_meas\":\"s\","
             "\"ic\":\"mdi:timer-outline\","
             "\"avty_t\":\"" MQTT_TOPIC_AVAILABLE "\","
             "%s}", HA_DEVICE_ID, dev_json);
    mqtt.publish(topic, payload, true);

    // Sensor: WiFi RSSI
    snprintf(topic, sizeof(topic),
             "%s/sensor/%s_rssi/config", HA_DISCOVERY_PREFIX, HA_DEVICE_ID);
    snprintf(payload, sizeof(payload),
             "{\"name\":\"HRV WiFi Signal\","
             "\"stat_t\":\"" MQTT_TOPIC_STATE "\","
             "\"val_tpl\":\"{{value_json.rssi}}\","
             "\"uniq_id\":\"%s_rssi\","
             "\"unit_of_meas\":\"dBm\","
             "\"ic\":\"mdi:wifi\","
             "\"ent_cat\":\"diagnostic\","
             "\"avty_t\":\"" MQTT_TOPIC_AVAILABLE "\","
             "%s}", HA_DEVICE_ID, dev_json);
    mqtt.publish(topic, payload, true);

    // Select: HRV Control
    snprintf(topic, sizeof(topic),
             "%s/select/%s_control/config", HA_DISCOVERY_PREFIX, HA_DEVICE_ID);
    snprintf(payload, sizeof(payload),
             "{\"name\":\"HRV Control\","
             "\"cmd_t\":\"" MQTT_TOPIC_COMMAND "\","
             "\"stat_t\":\"" MQTT_TOPIC_CMD_STATE "\","
             "\"uniq_id\":\"%s_control\","
             "\"ic\":\"mdi:hvac\","
             "\"options\":[\"Off\","
             "\"Fresh 1\",\"Fresh 2\",\"Fresh 3\",\"Fresh 4\",\"Fresh 5\","
             "\"Recirc 1\",\"Recirc 2\",\"Recirc 3\",\"Recirc 4\",\"Recirc 5\"],"
             "\"avty_t\":\"" MQTT_TOPIC_AVAILABLE "\","
             "%s}", HA_DEVICE_ID, dev_json);
    mqtt.publish(topic, payload, true);

    mqtt.publish(MQTT_TOPIC_CMD_STATE, current_cmd_label, true);
    Serial.println(F("HA discovery published"));
}

// ── State publishing ───────────────────────────────────────────────────────

static void mqtt_publish_state() {
    if (!mqtt.connected()) return;

    char payload[200];
    snprintf(payload, sizeof(payload),
             "{\"mode\":\"%s\",\"fan\":%d,\"uptime\":%lu,\"frames\":%lu,\"rssi\":%d}",
             hrv_mode_str(hrv_state.mode),
             hrv_state.fan_speed,
             millis() / 1000,
             hrv_state.frame_count,
             WiFi.RSSI());
    mqtt.publish(MQTT_TOPIC_STATE, payload);
}

// ── MQTT reconnect ─────────────────────────────────────────────────────────

static void mqtt_reconnect() {
    if (!wifi_connected) return;
    uint32_t now = millis();
    if ((now - last_mqtt_reconnect_ms) < MQTT_RECONNECT_MS) return;
    last_mqtt_reconnect_ms = now;

    Serial.print(F("MQTT connecting..."));
    if (mqtt.connect(OTA_HOSTNAME, mqtt_user, mqtt_pass,
                     MQTT_TOPIC_AVAILABLE, 0, true, "offline")) {
        Serial.println(F(" OK"));
        mqtt.publish(MQTT_TOPIC_AVAILABLE, "online", true);
        mqtt.subscribe(MQTT_TOPIC_COMMAND);
        mqtt_connected = true;

        if (!ha_discovery_sent) {
            mqtt_publish_ha_discovery();
            ha_discovery_sent = true;
        }

        mqtt_publish_state();
    } else {
        Serial.printf(" failed (rc=%d)\n", mqtt.state());
        mqtt_connected = false;
    }
}

// ── Public: MQTT loop ──────────────────────────────────────────────────────

void mqtt_loop() {
    if (!wifi_connected) return;

    if (!mqtt.connected()) {
        mqtt_connected = false;
        mqtt_reconnect();
        return;
    }

    mqtt.loop();

    uint32_t now = millis();
    bool state_changed = (hrv_state.mode     != prev_mqtt_state.mode) ||
                         (hrv_state.fan_speed != prev_mqtt_state.fan_speed);
    bool heartbeat = (now - last_mqtt_publish_ms) >= MQTT_HEARTBEAT_MS;

    if (state_changed || heartbeat) {
        mqtt_publish_state();
        last_mqtt_publish_ms = now;
        prev_mqtt_state.mode      = hrv_state.mode;
        prev_mqtt_state.fan_speed = hrv_state.fan_speed;
    }
}

// ── Public: WiFi setup ─────────────────────────────────────────────────────

void wifi_setup(Adafruit_SSD1306& display) {
    prefs.begin("lifebreach", true);
    prefs.getString("mqtt_srv", mqtt_server, sizeof(mqtt_server));
    prefs.getString("mqtt_usr", mqtt_user, sizeof(mqtt_user));
    prefs.getString("mqtt_pwd", mqtt_pass, sizeof(mqtt_pass));
    prefs.end();

    WiFiManagerParameter param_mqtt_server("mqtt_server", "MQTT Broker IP", mqtt_server, 40);
    WiFiManagerParameter param_mqtt_user("mqtt_user", "MQTT Username", mqtt_user, 20);
    WiFiManagerParameter param_mqtt_pass("mqtt_pass", "MQTT Password", mqtt_pass, 20);

    WiFiManager wm;
    wm.addParameter(&param_mqtt_server);
    wm.addParameter(&param_mqtt_user);
    wm.addParameter(&param_mqtt_pass);
    wm.setConfigPortalTimeout(WIFI_PORTAL_TIMEOUT);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 0);
    display.print(F("WiFi Setup"));
    display.setCursor(0, 16);
    display.print(F("Connect to AP:"));
    display.setTextSize(2);
    display.setCursor(0, 28);
    display.print(F("LifeBreach"));
    display.setTextSize(1);
    display.setCursor(0, 48);
    display.print(F("Then open 192.168.4.1"));
    display.display();

    bool connected = wm.autoConnect(WIFI_AP_NAME);

    if (connected) {
        wifi_connected = true;
        Serial.printf("WiFi connected: %s\n", WiFi.localIP().toString().c_str());

        strncpy(mqtt_server, param_mqtt_server.getValue(), sizeof(mqtt_server));
        strncpy(mqtt_user, param_mqtt_user.getValue(), sizeof(mqtt_user));
        strncpy(mqtt_pass, param_mqtt_pass.getValue(), sizeof(mqtt_pass));

        prefs.begin("lifebreach", false);
        prefs.putString("mqtt_srv", mqtt_server);
        prefs.putString("mqtt_usr", mqtt_user);
        prefs.putString("mqtt_pwd", mqtt_pass);
        prefs.end();

        if (strlen(mqtt_server) > 0) {
            mqtt.setServer(mqtt_server, 1883);
            mqtt.setCallback(mqtt_callback);
            mqtt.setBufferSize(512);
            Serial.printf("MQTT broker: %s\n", mqtt_server);
        }

        ArduinoOTA.setHostname(OTA_HOSTNAME);
        ArduinoOTA.setPassword(OTA_PASSWORD);
        ArduinoOTA.onStart([&display]() {
            display.clearDisplay();
            display.setTextSize(2);
            display.setCursor(4, 24);
            display.print(F("OTA UPDATE"));
            display.display();
        });
        ArduinoOTA.onProgress([&display](unsigned int progress, unsigned int total) {
            display.fillRect(14, 48, 100, 8, SSD1306_BLACK);
            display.drawRect(14, 48, 100, 8, SSD1306_WHITE);
            int w = (progress * 100) / total;
            display.fillRect(15, 49, w - 2, 6, SSD1306_WHITE);
            display.display();
        });
        ArduinoOTA.begin();
    } else {
        wifi_connected = false;
        Serial.println(F("WiFi not configured — running standalone"));
    }
}
