#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "driver/uart.h"
#include "logo_bitmap.h"
#include "hrv_protocol.h"

// ── OLED configuration ──────────────────────────────────────────────────────

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDR     0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ── HRV state ───────────────────────────────────────────────────────────────

static QueueHandle_t uart_queue;
static hrv_state_t   hrv_state = {MODE_UNKNOWN, 0, false, false, 0, 0, 0};
static hrv_state_t   prev_display_state = {MODE_UNKNOWN, 0, false, false, 0, 0, 0};

// Sliding buffer for frame sync
static uint8_t frame_buf[HRV_FRAME_LEN];
static uint8_t frame_buf_pos = 0;  // number of bytes collected (0-10)

// LCD command byte captured between frames
static uint8_t last_lcd_byte = 0xFF;
static bool    lcd_byte_valid = false;

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
    uart_driver_install(HRV_UART_NUM, HRV_BUF_SIZE, 0, 8, &uart_queue, 0);
    uart_set_line_inverse(HRV_UART_NUM, UART_SIGNAL_RXD_INV);
}

// ── Frame decoding ──────────────────────────────────────────────────────────

bool hrv_decode_frame(const uint8_t* buf, hrv_mode_t* mode, uint8_t* fan) {
    // Validate sync byte and tail
    if (buf[0] != HRV_SYNC_BYTE) return false;
    for (int i = 0; i < 5; i++) {
        if (buf[5 + i] != HRV_TAIL[i]) return false;
    }

    // Lookup (B1, B2, B3)
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
    return false;  // valid frame structure but unknown speed/mode combo
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
    // Shift sliding buffer left, insert new byte
    if (frame_buf_pos < HRV_FRAME_LEN) {
        frame_buf[frame_buf_pos++] = val;
    } else {
        memmove(frame_buf, frame_buf + 1, HRV_FRAME_LEN - 1);
        frame_buf[HRV_FRAME_LEN - 1] = val;
    }

    // Need full buffer to attempt decode
    if (frame_buf_pos < HRV_FRAME_LEN) return;

    hrv_mode_t mode;
    uint8_t    fan;

    if (hrv_decode_frame(frame_buf, &mode, &fan)) {
        hrv_state.mode          = mode;
        hrv_state.fan_speed     = fan;
        hrv_state.valid         = true;
        hrv_state.last_frame_ms = millis();
        hrv_state.frame_count++;

        // Debug: print decoded frame
        Serial.printf("#%lu  ", hrv_state.frame_count);
        for (int i = 0; i < HRV_FRAME_LEN; i++) {
            Serial.printf("%02X ", frame_buf[i]);
        }
        Serial.printf(" %s Fan %d", hrv_mode_str(mode), fan);

        // Print LCD byte if captured
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

        // Reset LCD capture for next cycle
        lcd_byte_valid = false;

        // Reset buffer position so next bytes start fresh
        frame_buf_pos = 0;
    } else {
        // This byte didn't complete a valid frame — could be LCD command byte
        last_lcd_byte  = val;
        lcd_byte_valid = true;
        hrv_state.error_count++;
    }
}

// ── UART polling ────────────────────────────────────────────────────────────

void hrv_uart_poll() {
    // Check for UART events (break detection for bathroom timer)
    uart_event_t event;
    while (xQueueReceive(uart_queue, &event, 0) == pdTRUE) {
        if (event.type == UART_BREAK) {
            hrv_state.bath_timer = true;
            bath_timer_start = millis();
            Serial.println(">>> BATHROOM TIMER DETECTED <<<");
        }
    }

    // Read available bytes
    uint8_t rx_buf[32];
    int len = uart_read_bytes(HRV_UART_NUM, rx_buf, sizeof(rx_buf), 0);
    for (int i = 0; i < len; i++) {
        hrv_process_byte(rx_buf[i]);
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

    // Check if anything changed
    bool state_changed = (hrv_state.mode      != prev_display_state.mode) ||
                         (hrv_state.fan_speed  != prev_display_state.fan_speed) ||
                         (hrv_state.valid      != prev_display_state.valid) ||
                         (hrv_state.bath_timer != prev_display_state.bath_timer);
    bool heartbeat = (now - last_display_ms) >= DISPLAY_HEARTBEAT_MS;

    if (!state_changed && !heartbeat) return;
    last_display_ms = now;

    display.clearDisplay();

    // ── Yellow zone (rows 0-15): header ──
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(19, 0);
    display.print(F("LIFEBREACH HRV"));
    display.setCursor(22, 8);
    display.print(F("--- MONITOR ---"));

    // ── Blue zone (rows 16-63) ──

    if (showing_bath_timer) {
        // Flash bathroom timer alert
        display.setTextSize(2);
        display.setCursor(16, 24);
        display.print(F("BATHROOM"));
        display.setCursor(28, 42);
        display.print(F("TIMER!"));
    } else if (signal_lost) {
        // No signal state
        display.setTextSize(2);
        display.setCursor(4, 20);
        display.print(F("WAITING..."));
        display.setTextSize(1);
        display.setCursor(16, 40);
        display.print(F("No HRV signal"));
    } else {
        // Normal state: mode + fan speed
        display.setTextSize(2);
        display.setCursor(4, 18);
        if (hrv_state.mode == MODE_FRESH) {
            display.print(F("FRESH AIR"));
        } else if (hrv_state.mode == MODE_RECIRC) {
            display.print(F("RECIRC"));
        } else {
            display.print(F("UNKNOWN"));
        }

        display.setCursor(4, 36);
        if (hrv_state.fan_speed == 0) {
            display.print(F("STANDBY"));
        } else {
            display.print(F("FAN: "));
            display.print(hrv_state.fan_speed);
        }

        // Speed bar
        display_draw_speed_bar(hrv_state.fan_speed);
    }

    // ── Status line (row 56) ──
    display.setTextSize(1);
    display.setCursor(0, 56);
    uint32_t uptime_sec = now / 1000;
    uint32_t up_min = uptime_sec / 60;
    if (up_min > 0) {
        display.printf("UP:%lum", up_min);
    } else {
        display.printf("UP:%lus", uptime_sec);
    }
    display.setCursor(50, 56);
    display.printf("FRM:%lu", hrv_state.frame_count);

    // Signal quality indicator
    display.setCursor(110, 56);
    if (signal_lost) {
        display.print(F("--"));
    } else {
        display.print(F("OK"));
    }

    display.display();

    // Clear bath timer flag after display period
    if (hrv_state.bath_timer && (now - bath_timer_start) >= BATH_TIMER_DISPLAY_MS) {
        hrv_state.bath_timer = false;
    }

    // Update previous state for change detection
    prev_display_state.mode      = hrv_state.mode;
    prev_display_state.fan_speed = hrv_state.fan_speed;
    prev_display_state.valid     = hrv_state.valid;
    prev_display_state.bath_timer = hrv_state.bath_timer;
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

    // Initialize HRV UART
    hrv_uart_init();
    Serial.println(F("LifeBreach HRV Monitor — Phase 1"));
    Serial.printf("UART%d @ %d baud, RX=GPIO%d (inverted)\n", HRV_UART_NUM, HRV_BAUD, HRV_RX_PIN);
}

// ── Main loop ───────────────────────────────────────────────────────────────

void loop() {
    hrv_uart_poll();
    display_update();
}
