#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "driver/uart.h"
#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include "logo_bitmap.h"
#include "hrv_protocol.h"
#include "config.h"

// ── OLED configuration ──────────────────────────────────────────────────────

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDR     0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ── WiFi / MQTT ─────────────────────────────────────────────────────────────

WiFiClient   espClient;
PubSubClient mqtt(espClient);
Preferences  prefs;

static char mqtt_server[40] = "";
static char mqtt_user[20]   = "";
static char mqtt_pass[20]   = "";
static bool wifi_connected  = false;
static bool mqtt_connected  = false;
static uint32_t last_mqtt_reconnect_ms = 0;
static uint32_t last_mqtt_publish_ms   = 0;
static bool     ha_discovery_sent      = false;

// ── HRV state ───────────────────────────────────────────────────────────────

static QueueHandle_t uart_queue;
static hrv_state_t   hrv_state = {MODE_UNKNOWN, 0, false, false, 0, 0, 0};
static hrv_state_t   prev_display_state = {MODE_UNKNOWN, 0, false, false, 0, 0, 0};
static hrv_state_t   prev_mqtt_state = {MODE_UNKNOWN, 0, false, false, 0, 0, 0};

// Sliding buffer for frame sync
static uint8_t frame_buf[HRV_FRAME_LEN];
static uint8_t frame_buf_pos = 0;

// LCD command byte captured between frames
static uint8_t last_lcd_byte = 0xFF;
static bool    lcd_byte_valid = false;
static bool    capture_next_lcd = false;

// TX command state — tx_command_byte is written by main loop, read by HRV task
static volatile uint8_t  tx_command_byte = 0xFF;
static volatile bool     tx_pending = false;
static volatile uint32_t tx_frame_decoded_us = 0;

// TX delay lookup — timing is part of the protocol, each command has a unique delay
// Microseconds from frame decode (D7 fully received) to TX byte start
// Pattern: ~500us (1 bit period) step per speed level
uint16_t get_tx_delay_us(uint8_t cmd_byte) {
    switch (cmd_byte) {
        case 0xCF: case 0xDF: return  560;  // Fan 1: 5.86ms from D7 start
        case 0xE7: case 0xEF: return 1060;  // Fan 2: 6.36ms from D7 start
        case 0xF3: case 0xF7: return 1570;  // Fan 3: 6.87ms from D7 start
        case 0xF9: case 0xFB: return 2070;  // Fan 4: 7.37ms from D7 start
        case 0xFC: case 0xFD: return 2580;  // Fan 5: 7.88ms from D7 start
        case 0xFF: default:   return 3590;  // Idle:  8.89ms from D7 start
    }
}

// Button state (BOOT button on GPIO0)
#define BUTTON_PIN       0
#define DEBOUNCE_MS      300
static uint32_t last_button_ms = 0;
static bool     tx_active = false;

// Test sequence state
static bool     test_running = false;
static uint8_t  test_step = 0;
static uint32_t test_step_ms = 0;
#define TEST_STEP_MS     10000

// Test sequence: cycle through all mode/fan combos
// {mode, fan, label}
struct test_entry_t {
    hrv_mode_t mode;
    uint8_t    fan;
    const char* label;
};

static const test_entry_t TEST_SEQUENCE[] = {
    {MODE_FRESH,   1, "Fresh Fan 1"},
    {MODE_FRESH,   2, "Fresh Fan 2"},
    {MODE_FRESH,   3, "Fresh Fan 3"},
    {MODE_FRESH,   4, "Fresh Fan 4"},
    {MODE_FRESH,   5, "Fresh Fan 5"},
    {MODE_RECIRC,  1, "Recirc Fan 1"},
    {MODE_RECIRC,  2, "Recirc Fan 2"},
    {MODE_RECIRC,  3, "Recirc Fan 3"},
    {MODE_RECIRC,  4, "Recirc Fan 4"},
    {MODE_RECIRC,  5, "Recirc Fan 5"},
    {MODE_UNKNOWN, 0, "Idle (Off)"},
};
#define TEST_SEQUENCE_COUNT (sizeof(TEST_SEQUENCE) / sizeof(TEST_SEQUENCE[0]))

// Display timing
static uint32_t last_display_ms  = 0;
static uint32_t bath_timer_start = 0;

#define DISPLAY_HEARTBEAT_MS  500
#define SIGNAL_LOST_MS        2000
#define BATH_TIMER_DISPLAY_MS 3000

// ── UART initialization (ESP-IDF) ───────────────────────────────────────────

void hrv_uart_init() {
    uart_config_t uart_config = {
        .baud_rate  = HRV_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(HRV_UART_NUM, &uart_config);
    uart_set_pin(HRV_UART_NUM, HRV_TX_PIN, HRV_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(HRV_UART_NUM, HRV_BUF_SIZE, 0, 32, &uart_queue, 0);
    uart_set_line_inverse(HRV_UART_NUM, UART_SIGNAL_RXD_INV);
    uart_set_rx_full_threshold(HRV_UART_NUM, 1);
    uart_set_rx_timeout(HRV_UART_NUM, 2);
}

// ── Frame decoding ──────────────────────────────────────────────────────────

bool hrv_decode_frame(const uint8_t* buf, hrv_mode_t* mode, uint8_t* fan) {
    if (buf[0] != HRV_SYNC_BYTE) return false;
    for (int i = 0; i < 5; i++) {
        if (buf[5 + i] != HRV_TAIL[i]) return false;
    }
    uint8_t b1 = buf[1], b2 = buf[2], b3 = buf[3];
    for (int i = 0; i < FRAME_LOOKUP_COUNT; i++) {
        if (FRAME_LOOKUP[i].b1 == b1 &&
            FRAME_LOOKUP[i].b2 == b2 &&
            FRAME_LOOKUP[i].b3 == b3) {
            *mode = FRAME_LOOKUP[i].mode;
            *fan  = FRAME_LOOKUP[i].fan_speed;
            return true;
        }
    }
    return false;
}

bool hrv_decode_lcd(uint8_t val, hrv_mode_t* mode, uint8_t* fan) {
    for (int i = 0; i < LCD_COMMANDS_COUNT; i++) {
        if (LCD_COMMANDS[i].byte_val == val) {
            *mode = LCD_COMMANDS[i].mode;
            *fan  = LCD_COMMANDS[i].fan_speed;
            return true;
        }
    }
    return false;
}

// ── Frame sync and byte processing ─────────────────────────────────────────

void hrv_process_byte(uint8_t val) {
    if (capture_next_lcd) {
        last_lcd_byte = val;
        lcd_byte_valid = true;
        capture_next_lcd = false;
    }

    if (frame_buf_pos < HRV_FRAME_LEN) {
        frame_buf[frame_buf_pos++] = val;
    } else {
        memmove(frame_buf, frame_buf + 1, HRV_FRAME_LEN - 1);
        frame_buf[HRV_FRAME_LEN - 1] = val;
    }

    if (frame_buf_pos < HRV_FRAME_LEN) return;

    hrv_mode_t mode;
    uint8_t    fan;

    if (hrv_decode_frame(frame_buf, &mode, &fan)) {
        hrv_state.mode          = mode;
        hrv_state.fan_speed     = fan;
        hrv_state.valid         = true;
        hrv_state.last_frame_ms = millis();
        hrv_state.frame_count++;

        Serial.printf("#%lu  ", hrv_state.frame_count);
        for (int i = 0; i < HRV_FRAME_LEN; i++) {
            Serial.printf("%02X ", frame_buf[i]);
        }
        Serial.printf(" %s Fan %d", hrv_mode_str(mode), fan);

        if (lcd_byte_valid) {
            hrv_mode_t lcd_mode;
            uint8_t    lcd_fan;
            Serial.printf("  LCD:%02X", last_lcd_byte);
            if (hrv_decode_lcd(last_lcd_byte, &lcd_mode, &lcd_fan)) {
                if (lcd_mode != MODE_UNKNOWN) {
                    Serial.printf(" (%s Fan %d)", hrv_mode_str(lcd_mode), lcd_fan);
                } else {
                    Serial.print(" (idle)");
                }
            }
        }
        Serial.println();

        lcd_byte_valid = false;
        capture_next_lcd = true;

        tx_pending = true;
        tx_frame_decoded_us = micros();

        frame_buf_pos = 0;
    } else {
        hrv_state.error_count++;
    }
}

// ── Button handling ─────────────────────────────────────────────────────────

void test_set_step(uint8_t step) {
    const test_entry_t* e = &TEST_SEQUENCE[step];
    tx_command_byte = hrv_lcd_command_byte(e->mode, e->fan);
    tx_active = (e->mode != MODE_UNKNOWN);
    Serial.printf("Test [%d/%d]: %s -> TX 0x%02X (delay %uus)\n",
                  step + 1, TEST_SEQUENCE_COUNT, e->label,
                  tx_command_byte, get_tx_delay_us(tx_command_byte));
}

void test_poll() {
    if (!test_running) return;
    uint32_t now = millis();
    if ((now - test_step_ms) >= TEST_STEP_MS) {
        test_step++;
        if (test_step >= TEST_SEQUENCE_COUNT) {
            test_running = false;
            tx_command_byte = 0xFF;
            tx_active = false;
            Serial.println(F("Test sequence complete"));
        } else {
            test_step_ms = now;
            test_set_step(test_step);
        }
    }
}

void button_poll() {
    if (digitalRead(BUTTON_PIN) == LOW) {
        uint32_t now = millis();
        if ((now - last_button_ms) > DEBOUNCE_MS) {
            // Check for long press (3s) to reset WiFi
            uint32_t press_start = millis();
            while (digitalRead(BUTTON_PIN) == LOW) {
                if (millis() - press_start > 3000) {
                    Serial.println(F("Long press — resetting WiFi config"));
                    display.clearDisplay();
                    display.setTextSize(1);
                    display.setTextColor(SSD1306_WHITE);
                    display.setCursor(0, 24);
                    display.print(F("WiFi reset..."));
                    display.setCursor(0, 36);
                    display.print(F("Rebooting"));
                    display.display();
                    WiFiManager wm;
                    wm.resetSettings();
                    delay(1000);
                    ESP.restart();
                }
            }

            // Short press — start/stop test sequence
            last_button_ms = millis();
            if (test_running) {
                test_running = false;
                tx_command_byte = 0xFF;
                tx_active = false;
                Serial.println(F("Test sequence stopped"));
            } else {
                test_running = true;
                test_step = 0;
                test_step_ms = millis();
                test_set_step(0);
            }
        }
    }
}

// ── HRV dedicated task (FreeRTOS, high priority, core 1) ────────────────────
// Handles UART RX, frame decode, and precision TX timing.
// Immune to OLED I2C, WiFi, MQTT blocking on the main loop.

static SemaphoreHandle_t tx_semaphore = NULL;

void hrv_tx_send() {
    uint8_t cmd = tx_command_byte;  // snapshot volatile
    uint16_t delay_us = get_tx_delay_us(cmd);

    // Spin-wait for exact timing — this task runs at high priority,
    // nothing except ISRs can preempt it
    while ((micros() - tx_frame_decoded_us) < delay_us) { /* spin */ }

    uart_write_bytes(HRV_UART_NUM, (const char*)&cmd, 1);
    tx_pending = false;
}

void hrv_task(void* param) {
    for (;;) {
        // Check for UART break events (bathroom timer)
        uart_event_t event;
        while (xQueueReceive(uart_queue, &event, 0) == pdTRUE) {
            if (event.type == UART_BREAK) {
                hrv_state.bath_timer = true;
                bath_timer_start = millis();
            }
        }

        // Block until a byte arrives (yields CPU efficiently via semaphore).
        // Returns immediately when data is available — near-zero latency.
        uint8_t byte;
        if (uart_read_bytes(HRV_UART_NUM, &byte, 1, pdMS_TO_TICKS(1)) == 1) {
            hrv_process_byte(byte);

            // Drain any remaining buffered bytes without waiting
            uint8_t rx_buf[31];
            int len = uart_read_bytes(HRV_UART_NUM, rx_buf, sizeof(rx_buf), 0);
            for (int i = 0; i < len; i++) {
                hrv_process_byte(rx_buf[i]);
            }
        }

        // TX: spin-wait for precise timing then send
        if (tx_pending) {
            hrv_tx_send();
        }
    }
}

// ── WiFi + MQTT ─────────────────────────────────────────────────────────────

// Command option strings — must match HA select discovery options exactly
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

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    char buf[32];
    unsigned int len = min(length, (unsigned int)(sizeof(buf) - 1));
    memcpy(buf, payload, len);
    buf[len] = '\0';

    Serial.printf("MQTT cmd: %s\n", buf);

    // Match against known options
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

            // Publish command state back so HA select reflects the choice
            mqtt.publish(MQTT_TOPIC_CMD_STATE, opt->label, true);
            return;
        }
    }

    Serial.printf("MQTT: unknown command '%s'\n", buf);
}

void mqtt_publish_ha_discovery() {
    char topic[128];
    char payload[512];

    // Device JSON fragment (shared by all entities)
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

    // Select: HRV Control (the controllable entity)
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

    // Publish initial command state
    mqtt.publish(MQTT_TOPIC_CMD_STATE, current_cmd_label, true);

    Serial.println(F("HA discovery published"));
}

void mqtt_publish_state() {
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

void mqtt_reconnect() {
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

        // Publish current state immediately
        mqtt_publish_state();
    } else {
        Serial.printf(" failed (rc=%d)\n", mqtt.state());
        mqtt_connected = false;
    }
}

void mqtt_loop() {
    if (!wifi_connected) return;

    if (!mqtt.connected()) {
        mqtt_connected = false;
        mqtt_reconnect();
        return;
    }

    mqtt.loop();

    // Publish on state change or heartbeat
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

// ── WiFi setup ──────────────────────────────────────────────────────────────

void wifi_setup() {
    // Load saved MQTT credentials
    prefs.begin("lifebreach", true);  // read-only
    prefs.getString("mqtt_srv", mqtt_server, sizeof(mqtt_server));
    prefs.getString("mqtt_usr", mqtt_user, sizeof(mqtt_user));
    prefs.getString("mqtt_pwd", mqtt_pass, sizeof(mqtt_pass));
    prefs.end();

    // WiFiManager custom parameters
    WiFiManagerParameter param_mqtt_server("mqtt_server", "MQTT Broker IP", mqtt_server, 40);
    WiFiManagerParameter param_mqtt_user("mqtt_user", "MQTT Username", mqtt_user, 20);
    WiFiManagerParameter param_mqtt_pass("mqtt_pass", "MQTT Password", mqtt_pass, 20);

    WiFiManager wm;
    wm.addParameter(&param_mqtt_server);
    wm.addParameter(&param_mqtt_user);
    wm.addParameter(&param_mqtt_pass);
    wm.setConfigPortalTimeout(WIFI_PORTAL_TIMEOUT);

    // Show portal status on OLED
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

        // Save MQTT params if they changed
        strncpy(mqtt_server, param_mqtt_server.getValue(), sizeof(mqtt_server));
        strncpy(mqtt_user, param_mqtt_user.getValue(), sizeof(mqtt_user));
        strncpy(mqtt_pass, param_mqtt_pass.getValue(), sizeof(mqtt_pass));

        prefs.begin("lifebreach", false);  // read-write
        prefs.putString("mqtt_srv", mqtt_server);
        prefs.putString("mqtt_usr", mqtt_user);
        prefs.putString("mqtt_pwd", mqtt_pass);
        prefs.end();

        // Configure MQTT
        if (strlen(mqtt_server) > 0) {
            mqtt.setServer(mqtt_server, 1883);
            mqtt.setCallback(mqtt_callback);
            mqtt.setBufferSize(512);  // for HA discovery payloads
            Serial.printf("MQTT broker: %s\n", mqtt_server);
        }

        // OTA setup
        ArduinoOTA.setHostname(OTA_HOSTNAME);
        ArduinoOTA.setPassword(OTA_PASSWORD);
        ArduinoOTA.onStart([]() {
            display.clearDisplay();
            display.setTextSize(2);
            display.setCursor(4, 24);
            display.print(F("OTA UPDATE"));
            display.display();
        });
        ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
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

// ── Display rendering ───────────────────────────────────────────────────────

void display_draw_speed_bar(uint8_t speed) {
    int bar_x = 14;
    int bar_y = 48;
    int bar_w = 100;
    int bar_h = 8;
    display.drawRect(bar_x, bar_y, bar_w, bar_h, SSD1306_WHITE);
    int fill_w = (speed * bar_w) / 5;
    if (fill_w > 0) {
        display.fillRect(bar_x + 1, bar_y + 1, fill_w - 2, bar_h - 2, SSD1306_WHITE);
    }
}

void display_update() {
    uint32_t now = millis();
    bool signal_lost = (hrv_state.last_frame_ms == 0) ||
                       ((now - hrv_state.last_frame_ms) > SIGNAL_LOST_MS);
    bool showing_bath_timer = hrv_state.bath_timer &&
                              ((now - bath_timer_start) < BATH_TIMER_DISPLAY_MS);

    static bool prev_tx_active = false;
    static uint8_t prev_test_step = 0xFF;
    bool state_changed = (hrv_state.mode      != prev_display_state.mode) ||
                         (hrv_state.fan_speed  != prev_display_state.fan_speed) ||
                         (hrv_state.valid      != prev_display_state.valid) ||
                         (hrv_state.bath_timer != prev_display_state.bath_timer) ||
                         (tx_active            != prev_tx_active) ||
                         (test_step            != prev_test_step);
    bool heartbeat = (now - last_display_ms) >= DISPLAY_HEARTBEAT_MS;

    if (!state_changed && !heartbeat) return;
    last_display_ms = now;

    display.clearDisplay();

    // ── Yellow zone (rows 0-15): header ──
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    if (tx_active && test_running) {
        display.printf("CMD: %s", TEST_SEQUENCE[test_step].label);
    } else if (tx_active) {
        // MQTT or other external command active
        hrv_mode_t cmd_mode;
        uint8_t cmd_fan;
        if (hrv_decode_lcd(tx_command_byte, &cmd_mode, &cmd_fan) && cmd_mode != MODE_UNKNOWN) {
            display.printf("CMD: %s Fan %d", hrv_mode_str(cmd_mode), cmd_fan);
        } else {
            display.print(F("CMD: Idle"));
        }
    } else {
        display.setCursor(19, 0);
        display.print(F("LIFEBREACH HRV"));
    }
    display.setCursor(0, 8);
    if (wifi_connected) {
        display.print(WiFi.localIP().toString().c_str());
        if (mqtt_connected) {
            display.print(F(" MQTT"));
        }
    } else {
        display.print(F("NO WIFI"));
    }

    // ── Blue zone (rows 16-63) ──

    if (showing_bath_timer) {
        display.setTextSize(2);
        display.setCursor(16, 24);
        display.print(F("BATHROOM"));
        display.setCursor(28, 42);
        display.print(F("TIMER!"));
    } else if (signal_lost) {
        display.setTextSize(2);
        display.setCursor(4, 20);
        display.print(F("WAITING..."));
        display.setTextSize(1);
        display.setCursor(16, 40);
        display.print(F("No HRV signal"));
    } else {
        display.setTextSize(2);
        display.setCursor(4, 16);
        if (hrv_state.mode == MODE_FRESH) {
            display.print(F("FRESH AIR"));
        } else if (hrv_state.mode == MODE_RECIRC) {
            display.print(F("RECIRC"));
        } else {
            display.print(F("UNKNOWN"));
        }

        display.setCursor(4, 32);
        if (hrv_state.fan_speed == 0) {
            display.print(F("STANDBY"));
        } else {
            display.print(F("FAN: "));
            display.print(hrv_state.fan_speed);
        }

        display_draw_speed_bar(hrv_state.fan_speed);
    }

    // ── Status line (row 57) ──
    display.setTextSize(1);
    display.setCursor(0, 57);
    uint32_t uptime_sec = now / 1000;
    uint32_t up_min = uptime_sec / 60;
    uint32_t up_hrs = up_min / 60;
    uint32_t up_days = up_hrs / 24;
    if (up_days > 0) {
        display.printf("UP:%lu.%lud", up_days, (up_hrs % 24) * 10 / 24);
    } else if (up_min >= 50) {
        display.printf("UP:%lu.%luh", up_hrs, (up_min % 60) * 10 / 60);
    } else if (up_min > 0) {
        display.printf("UP:%lum", up_min);
    } else {
        display.printf("UP:%lus", uptime_sec);
    }
    display.setCursor(50, 57);
    if (hrv_state.frame_count > 9999) {
        display.printf("FRM:%lu.%luk", hrv_state.frame_count / 1000, (hrv_state.frame_count % 1000) / 100);
    } else {
        display.printf("FRM:%lu", hrv_state.frame_count);
    }

    display.setCursor(110, 57);
    if (signal_lost) {
        display.print(F("--"));
    } else if (tx_active) {
        display.print(F("TX"));
    } else {
        display.print(F("RX"));
    }

    display.display();

    if (hrv_state.bath_timer && (now - bath_timer_start) >= BATH_TIMER_DISPLAY_MS) {
        hrv_state.bath_timer = false;
    }

    prev_display_state.mode      = hrv_state.mode;
    prev_display_state.fan_speed = hrv_state.fan_speed;
    prev_display_state.valid     = hrv_state.valid;
    prev_display_state.bath_timer = hrv_state.bath_timer;
    prev_tx_active = tx_active;
    prev_test_step = test_step;
}

// ── Setup ───────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);

    // Initialize OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }

    // Splash screen
    display.clearDisplay();
    display.drawBitmap(0, 0, lifebreach_logo, LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 48);
    display.print(F("Build "));
    display.print(F(__DATE__));
    display.setCursor(0, 56);
    display.print(F(__TIME__));
    display.display();
    delay(3000);

    // WiFi + MQTT + OTA
    wifi_setup();

    // Initialize HRV UART
    hrv_uart_init();

    // Launch HRV task on core 1 at high priority (19 of 24)
    // This task handles UART RX, frame decode, and precision TX timing
    xTaskCreatePinnedToCore(
        hrv_task,       // task function
        "hrv_task",     // name
        4096,           // stack size
        NULL,           // parameter
        19,             // priority (high — only system tasks above)
        NULL,           // task handle
        1               // core 1
    );

    // Initialize button
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    Serial.println(F("LifeBreach HRV — WiFi + MQTT enabled (FreeRTOS TX)"));
    Serial.printf("UART%d @ %d baud, RX=GPIO%d (inverted), TX=GPIO%d\n",
                  HRV_UART_NUM, HRV_BAUD, HRV_RX_PIN, HRV_TX_PIN);
}

// ── Main loop ───────────────────────────────────────────────────────────────

void loop() {
    // HRV UART + TX timing handled by hrv_task on core 1
    button_poll();
    test_poll();
    mqtt_loop();
    if (wifi_connected) ArduinoOTA.handle();
    display_update();
}
