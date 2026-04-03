#ifndef HRV_PROTOCOL_H
#define HRV_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

// ── Protocol constants ───────────────────────────────────────────────────────

#define HRV_FRAME_LEN   10
#define HRV_SYNC_BYTE   0xBF
#define HRV_FRESH        0x5E
#define HRV_RECIRC       0x5D

static const uint8_t HRV_TAIL[5] = {0x3B, 0x2F, 0xF7, 0xEF, 0xD7};

// ── UART configuration ──────────────────────────────────────────────────────

#define HRV_BAUD        2000
#define HRV_UART_NUM    UART_NUM_2
#define HRV_RX_PIN      16
#define HRV_TX_PIN      17
#define HRV_BUF_SIZE    256

// ── Data types ──────────────────────────────────────────────────────────────

typedef enum {
    MODE_UNKNOWN = 0,
    MODE_FRESH,
    MODE_RECIRC
} hrv_mode_t;

typedef struct {
    hrv_mode_t mode;
    uint8_t    fan_speed;      // 0-5
    bool       valid;          // true if last decode succeeded
    bool       bath_timer;     // true if bathroom timer pulse detected
    uint32_t   last_frame_ms;  // millis() of last valid frame
    uint32_t   frame_count;    // total valid frames received
    uint32_t   error_count;    // bytes that didn't form valid frames
} hrv_state_t;

// ── Frame lookup table ──────────────────────────────────────────────────────
// Decode key: (B1, B2, B3) from 10-byte HRV frame
// Source: hrv_live.py lines 39-52

typedef struct {
    uint8_t    b1;
    uint8_t    b2;
    uint8_t    b3;
    hrv_mode_t mode;
    uint8_t    fan_speed;
} frame_lookup_t;

static const frame_lookup_t FRAME_LOOKUP[] = {
    {0xAF, 0x9F, HRV_FRESH,  MODE_FRESH,  0},
    {0xAF, 0x99, HRV_FRESH,  MODE_FRESH,  1},
    {0xAF, 0x95, HRV_FRESH,  MODE_FRESH,  2},
    {0xAF, 0x91, HRV_FRESH,  MODE_FRESH,  3},
    {0xAE, 0x9D, HRV_FRESH,  MODE_FRESH,  4},
    {0xAE, 0x99, HRV_FRESH,  MODE_FRESH,  5},
    {0xAF, 0x9F, HRV_RECIRC, MODE_RECIRC, 0},
    {0xAF, 0x9B, HRV_RECIRC, MODE_RECIRC, 1},
    {0xAF, 0x97, HRV_RECIRC, MODE_RECIRC, 2},
    {0xAF, 0x93, HRV_RECIRC, MODE_RECIRC, 3},
    {0xAE, 0x9F, HRV_RECIRC, MODE_RECIRC, 4},
    {0xAE, 0x9B, HRV_RECIRC, MODE_RECIRC, 5},
};
#define FRAME_LOOKUP_COUNT 12

// ── LCD command lookup table ────────────────────────────────────────────────
// Source: hrv_live.py lines 92-104

typedef struct {
    uint8_t    byte_val;
    hrv_mode_t mode;       // MODE_UNKNOWN for idle/no command
    uint8_t    fan_speed;
} lcd_command_t;

static const lcd_command_t LCD_COMMANDS[] = {
    {0xFF, MODE_UNKNOWN, 0},  // idle
    {0xDF, MODE_FRESH,   1},
    {0xEF, MODE_FRESH,   2},
    {0xF7, MODE_FRESH,   3},
    {0xFB, MODE_FRESH,   4},
    {0xFD, MODE_FRESH,   5},
    {0xCF, MODE_RECIRC,  1},
    {0xE7, MODE_RECIRC,  2},
    {0xF3, MODE_RECIRC,  3},
    {0xF9, MODE_RECIRC,  4},
    {0xFC, MODE_RECIRC,  5},
};
#define LCD_COMMANDS_COUNT 11

// ── Helper: mode to string ──────────────────────────────────────────────────

static inline const char* hrv_mode_str(hrv_mode_t mode) {
    switch (mode) {
        case MODE_FRESH:  return "Fresh";
        case MODE_RECIRC: return "Recirc";
        default:          return "Unknown";
    }
}

#endif // HRV_PROTOCOL_H
