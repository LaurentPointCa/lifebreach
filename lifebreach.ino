#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include "logo_bitmap.h"
#include "hrv_protocol.h"
#include "hrv_uart.h"
#include "wifi_mqtt.h"
#include "config.h"

// ── OLED configuration ──────────────────────────────────────────────────────

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDR     0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ── Button state (BOOT button on GPIO0) ────────────────────────────────────

#define BUTTON_PIN       0
#define DEBOUNCE_MS      300
static uint32_t last_button_ms = 0;

// ── Test sequence ──────────────────────────────────────────────────────────

static bool     test_running = false;
static uint8_t  test_step = 0;
static uint32_t test_step_ms = 0;
#define TEST_STEP_MS     10000

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

// ── Display timing ─────────────────────────────────────────────────────────

static uint32_t last_display_ms = 0;
static hrv_state_t prev_display_state = {MODE_UNKNOWN, 0, false, false, 0, 0, 0};

#define DISPLAY_HEARTBEAT_MS  500
#define SIGNAL_LOST_MS        2000
#define BATH_TIMER_DISPLAY_MS 3000

// ── Button handling ────────────────────────────────────────────────────────

void test_set_step(uint8_t step) {
    const test_entry_t* e = &TEST_SEQUENCE[step];
    tx_command_byte = hrv_lcd_command_byte(e->mode, e->fan);
    tx_active = (e->mode != MODE_UNKNOWN);
#if HRV_DEBUG
    Serial.printf("Test [%d/%d]: %s -> TX 0x%02X (delay %uus)\n",
                  step + 1, TEST_SEQUENCE_COUNT, e->label,
                  tx_command_byte, get_tx_delay_us(tx_command_byte));
#endif
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
#if HRV_DEBUG
            Serial.println(F("Test sequence complete"));
#endif
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
#if HRV_DEBUG
                    Serial.println(F("Long press — resetting WiFi config"));
#endif
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
#if HRV_DEBUG
                Serial.println(F("Test sequence stopped"));
#endif
            } else {
                test_running = true;
                test_step = 0;
                test_step_ms = millis();
                test_set_step(0);
            }
        }
    }
}

// ── Display rendering ──────────────────────────────────────────────────────

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
    bool showing_lcd_bath_timer = lcd_bath_timer &&
                                  ((now - lcd_bath_timer_start) < BATH_TIMER_DISPLAY_MS);

    static bool prev_tx_active = false;
    static uint8_t prev_test_step = 0xFF;
    static bool prev_lcd_bath_timer = false;
    bool state_changed = (hrv_state.mode      != prev_display_state.mode) ||
                         (hrv_state.fan_speed  != prev_display_state.fan_speed) ||
                         (hrv_state.valid      != prev_display_state.valid) ||
                         (hrv_state.bath_timer != prev_display_state.bath_timer) ||
                         (lcd_bath_timer       != prev_lcd_bath_timer) ||
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

    if (signal_lost) {
        display.setTextSize(2);
        display.setCursor(4, 20);
        display.print(F("WAITING..."));
        display.setTextSize(1);
        display.setCursor(16, 40);
        display.print(F("No HRV signal"));
    } else {
        display.setTextSize(2);

        // Line 1 (row 16): LCD panel command — overridden by LCD-side bathroom timer
        display.setCursor(4, 16);
        if (showing_lcd_bath_timer) {
            display.print(F("TIMER!"));
        } else if (lcd_rx_valid) {
            hrv_mode_t lcd_mode;
            uint8_t    lcd_fan;
            if (hrv_decode_lcd(lcd_rx_last_cmd, &lcd_mode, &lcd_fan) && lcd_mode != MODE_UNKNOWN) {
                display.printf("%-6s|%2c", hrv_mode_str(lcd_mode),
                               lcd_fan > 0 ? ('0' + lcd_fan) : '-');
            } else {
                display.print(F("IDLE  | -"));
            }
        } else {
            display.print(F("LCD  ?"));
        }

        // Line 2 (row 32): HRV status — overridden by bathroom timer banner
        display.setCursor(4, 32);
        if (showing_bath_timer) {
            display.print(F("TIMER!"));
        } else if (hrv_state.mode != MODE_UNKNOWN) {
            display.printf("%-6s|%2c", hrv_mode_str(hrv_state.mode),
                           hrv_state.fan_speed > 0 ? ('0' + hrv_state.fan_speed) : '-');
        } else {
            display.print(F("  ??  | -"));
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
    if (lcd_bath_timer && (now - lcd_bath_timer_start) >= BATH_TIMER_DISPLAY_MS) {
        lcd_bath_timer = false;
    }
    prev_lcd_bath_timer = lcd_bath_timer;

    prev_display_state.mode      = hrv_state.mode;
    prev_display_state.fan_speed = hrv_state.fan_speed;
    prev_display_state.valid     = hrv_state.valid;
    prev_display_state.bath_timer = hrv_state.bath_timer;
    prev_tx_active = tx_active;
    prev_test_step = test_step;
}

// ── Setup ──────────────────────────────────────────────────────────────────

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
    wifi_setup(display);

    // Initialize UARTs
    hrv_uart_init();
    lcd_uart_init();

    // Launch HRV task on core 1 at high priority (19 of 24)
    xTaskCreatePinnedToCore(
        hrv_task,       // task function
        "hrv_task",     // name
        4096,           // stack size
        NULL,           // parameter
        19,             // priority
        NULL,           // task handle
        1               // core 1
    );

    // Initialize button
    pinMode(BUTTON_PIN, INPUT_PULLUP);

#if HRV_DEBUG
    Serial.println(F("LifeBreach HRV — WiFi + MQTT + LCD relay (FreeRTOS TX) [DEBUG]"));
    Serial.printf("HRV UART%d @ %d baud, RX=GPIO%d (inv), TX=GPIO%d\n",
                  HRV_UART_NUM, HRV_BAUD, HRV_RX_PIN, HRV_TX_PIN);
    Serial.printf("LCD UART%d @ %d baud, TX=GPIO%d, RX=GPIO%d\n",
                  LCD_UART_NUM, HRV_BAUD, LCD_TX_PIN, LCD_RX_PIN);
#endif
}

// ── Main loop ──────────────────────────────────────────────────────────────

void loop() {
    button_poll();
    test_poll();
    lcd_rx_poll();
    mqtt_loop();
    if (wifi_connected) ArduinoOTA.handle();
    display_update();
}
