#include <Arduino.h>
#include "driver/uart.h"
#include "hrv_uart.h"
#include "hrv_protocol.h"

// ── Shared state ───────────────────────────────────────────────────────────

hrv_state_t   hrv_state = {MODE_UNKNOWN, 0, false, false, 0, 0, 0};
uint32_t      bath_timer_start = 0;

volatile uint8_t  tx_command_byte = 0xFF;
volatile bool     tx_pending = false;
volatile uint32_t tx_frame_decoded_us = 0;
bool              tx_active = false;

// ── Private state ──────────────────────────────────────────────────────────

static QueueHandle_t uart_queue;

static uint8_t frame_buf[HRV_FRAME_LEN];
static uint8_t frame_buf_pos = 0;

static uint8_t last_lcd_byte = 0xFF;
static bool    lcd_byte_valid = false;
static bool    capture_next_lcd = false;

// LCD relay state — sends one byte every 20ms to match HRV timing
static uint8_t  lcd_relay_buf[HRV_FRAME_LEN];
static uint8_t  lcd_relay_pos = HRV_FRAME_LEN;  // >= FRAME_LEN means idle
static uint32_t lcd_relay_last_us = 0;
#define LCD_BYTE_INTERVAL_US  20000  // 20ms start-to-start

// ── TX delay lookup ────────────────────────────────────────────────────────
// Timing is part of the protocol — each command has a unique delay.
// Microseconds from frame decode (D7 fully received) to TX byte start.
// Pattern: ~500us (1 bit period) step per speed level.

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

// ── UART initialization ────────────────────────────────────────────────────

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

void lcd_uart_init() {
    uart_config_t uart_config = {
        .baud_rate  = HRV_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(LCD_UART_NUM, &uart_config);
    uart_set_pin(LCD_UART_NUM, LCD_TX_PIN, LCD_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(LCD_UART_NUM, LCD_BUF_SIZE, LCD_BUF_SIZE, 0, NULL, 0);
    uart_set_line_inverse(LCD_UART_NUM, UART_SIGNAL_RXD_INV);
}

// ── Frame decoding ─────────────────────────────────────────────────────────

static bool hrv_decode_frame(const uint8_t* buf, hrv_mode_t* mode, uint8_t* fan) {
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

// ── Frame sync and byte processing ────────────────────────────────────────

static void hrv_process_byte(uint8_t val) {
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

        // WARNING: Debug output MUST stay before tx_frame_decoded_us capture.
        // The TX delay values (560-3590us) were calibrated with ~0.44ms of
        // Serial.printf overhead between frame decode and timestamp capture.
        // Moving, adding, or removing prints here shifts ALL command timing.
        // If you change this block, recalibrate get_tx_delay_us() values.
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

        // TX timestamp — calibrated delay values depend on this being
        // captured after the Serial.printf block above
        tx_pending = true;
        tx_frame_decoded_us = micros();

        // Start forwarding HRV frame to LCD panel — one byte every 20ms
        memcpy(lcd_relay_buf, frame_buf, HRV_FRAME_LEN);
        lcd_relay_pos = 1;
        lcd_relay_last_us = tx_frame_decoded_us;
        uart_tx_chars(LCD_UART_NUM, (const char*)&lcd_relay_buf[0], 1);

        frame_buf_pos = 0;
    } else {
        hrv_state.error_count++;
    }
}

// ── TX send ────────────────────────────────────────────────────────────────

static void hrv_tx_send() {
    uint8_t cmd = tx_command_byte;  // snapshot volatile
    uint16_t delay_us = get_tx_delay_us(cmd);

    // Spin-wait for exact timing — this task runs at high priority,
    // nothing except ISRs can preempt it
    while ((micros() - tx_frame_decoded_us) < delay_us) { /* spin */ }

    uart_write_bytes(HRV_UART_NUM, (const char*)&cmd, 1);
    tx_pending = false;
}

// ── HRV dedicated task (FreeRTOS, high priority, core 1) ───────────────────
// Handles UART RX, frame decode, and precision TX timing.
// Immune to OLED I2C, WiFi, MQTT blocking on the main loop.

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

        // LCD relay: send next byte when 20ms has elapsed
        if (lcd_relay_pos < HRV_FRAME_LEN) {
            if ((micros() - lcd_relay_last_us) >= LCD_BYTE_INTERVAL_US) {
                uart_tx_chars(LCD_UART_NUM, (const char*)&lcd_relay_buf[lcd_relay_pos], 1);
                lcd_relay_pos++;
                lcd_relay_last_us += LCD_BYTE_INTERVAL_US;
            }
        }
    }
}
