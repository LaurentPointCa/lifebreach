#ifndef HRV_UART_H
#define HRV_UART_H

#include "hrv_protocol.h"

// ── Shared state (written by hrv_task, read by main loop) ──────────────────

extern hrv_state_t   hrv_state;
extern uint32_t      bath_timer_start;

// TX command state (written by main loop / MQTT callback, read by hrv_task)
extern volatile uint8_t  tx_command_byte;
extern volatile bool     tx_pending;
extern volatile uint32_t tx_frame_decoded_us;
extern bool              tx_active;

// LCD RX state (written by lcd_rx_poll, read by main loop for display)
extern uint8_t  lcd_rx_last_cmd;
extern bool     lcd_rx_valid;

// ── Public API ─────────────────────────────────────────────────────────────

void     hrv_uart_init();
void     lcd_uart_init();
void     hrv_task(void* param);
void     lcd_rx_poll();
uint16_t get_tx_delay_us(uint8_t cmd_byte);
bool     hrv_decode_lcd(uint8_t val, hrv_mode_t* mode, uint8_t* fan);

#endif // HRV_UART_H
