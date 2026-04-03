# TX Timing Findings — ESP32 vs Real LCD (DXPL02)

## Status: TIMING VALUES CORRECT, JITTER FIX NEEDED

Per-command delay values are correct and produce the right HRV behavior when the
byte lands on time. However, OLED I2C writes in `display_update()` block the main
loop for 15-20ms, causing periodic collisions and timing errors that corrupt
~10% of frames.

## Protocol: Timing encodes fan speed

Every command has a unique, precise delay from D7 (last frame byte) start:

| LCD byte | Command | Delay from D7 start | ESP32 TX_DELAY_US |
|----------|---------|---------------------|-------------------|
| 0xCF/0xDF | Fan 1 | **5.86ms** | **360** |
| 0xE7/0xEF | Fan 2 | **6.36ms** | **860** |
| 0xF3/0xF7 | Fan 3 | **6.87ms** | **1370** |
| 0xF9/0xFB | Fan 4 | **7.37ms** | **1870** |
| 0xFC/0xFD | Fan 5 | **7.88ms** | **2380** |
| 0xFF | Idle | **8.89ms** | **3390** |

Pattern: ~500us (1 bit period at 2000 baud) step per speed level.
TX_DELAY_US = microseconds from frame decode to TX start. These values are
verified correct (account for ~0.5ms UART/ISR latency in frame decode).

```c
uint16_t get_tx_delay_us(uint8_t cmd_byte) {
    switch (cmd_byte) {
        case 0xCF: case 0xDF: return  360;  // Fan 1
        case 0xE7: case 0xEF: return  860;  // Fan 2
        case 0xF3: case 0xF7: return 1370;  // Fan 3
        case 0xF9: case 0xFB: return 1870;  // Fan 4
        case 0xFC: case 0xFD: return 2380;  // Fan 5
        case 0xFF: default:   return 3390;  // Idle
    }
}
```

## Current Problem: OLED I2C blocks TX timing

### Evidence from `hrv-esp-cycletest.vcd`

**57 out of 525 frames** (11%) have bus collisions — the ESP32 TX byte lands in the
middle of the next HRV frame, producing 402ms double-cycles with garbled data.

Additionally, 13 non-collision frames have timing errors of +3 to +8ms, pushing the
byte into the wrong speed slot.

### Failure pattern

The collisions occur every ~4-5 frames (~800-1000ms), correlating with the
`DISPLAY_HEARTBEAT_MS = 500` OLED refresh interval. When `display_update()` fires
during the TX-critical window (360-3390us after frame decode), it blocks the main
loop for 15-20ms (full SSD1306 I2C frame transfer at 400kHz), causing:

- **Mild**: TX fires 5-8ms late → byte lands in wrong timing slot (e.g., Fan 1 byte
  arrives at Fan 3 timing)
- **Severe**: TX fires 15+ ms late → byte collides with next HRV frame → frame
  destroyed, 402ms double-cycle

### Impact on cycle test

| Command | Expected | Got | Cause |
|---------|----------|-----|-------|
| Fresh F1 | Fresh F1 | Fresh F1 (5.4s delay) | OK but slow — some frames lost to collisions |
| Fresh F2 | Fresh F2 | **stayed F1** | Collisions wiped out most F2 command frames |
| Fresh F3 | Fresh F3 | Fresh F3 (6.0s delay) | OK but slow |
| Fresh F4 | Fresh F4 | Fresh F4 (2.0s) | OK |
| Fresh F5 | Fresh F5 | Fresh F5 (6.8s delay) | OK but slow |
| Recirc F1 | Recirc F1 | **stayed Fresh F5** | Collisions wiped out R1 command frames |
| Recirc F2 | Recirc F2 | Recirc F2 (4.0s) | OK but slow |
| Recirc F3-5 | OK | OK | 2-6s delays |
| Idle | Fresh F0 | Fresh F0 (2.0s) | OK |

Fresh F2 and Recirc F1 failed completely. Most others worked but with 4-7s response
times (should be 2s) because only ~90% of frames carried valid commands.

## Required Fix: Move TX to a hardware timer ISR

The TX byte MUST be sent from a context that cannot be blocked by I2C. The OLED
update takes 15-20ms and the TX-critical window is as short as 360us — polling in
`loop()` will never be reliable enough.

### Recommended: ESP32 hardware timer one-shot

```c
#include "driver/gptimer.h"

static gptimer_handle_t tx_timer = NULL;
static volatile uint8_t tx_isr_byte = 0xFF;

// ISR — fires at exactly TX_DELAY_US after frame decode
static bool IRAM_ATTR tx_timer_isr(gptimer_handle_t timer,
                                    const gptimer_alarm_event_data_t *edata,
                                    void *user_ctx) {
    uart_write_bytes(HRV_UART_NUM, (const char*)&tx_isr_byte, 1);
    return false;  // no need to yield
}

// Call from hrv_process_byte() after successful frame decode:
void schedule_tx(uint8_t cmd_byte) {
    tx_isr_byte = cmd_byte;
    uint16_t delay = get_tx_delay_us(cmd_byte);

    gptimer_set_raw_count(tx_timer, 0);
    gptimer_alarm_config_t alarm = {
        .alarm_count = delay,
        .flags.auto_reload_on_alarm = false,
    };
    gptimer_set_alarm_action(tx_timer, &alarm);
    gptimer_start(tx_timer);
}
```

This guarantees the TX byte fires at the exact microsecond regardless of what
the main loop is doing (OLED writes, serial debug prints, WiFi, etc.).

### Alternative: FreeRTOS high-priority task

Dedicate a task at priority > configMAX_PRIORITIES-2 that blocks on a semaphore.
Frame decode gives the semaphore, the task spin-waits on `micros()` then sends.
Less precise than a hardware timer but simpler to implement.

### Minimum viable: Block OLED during TX window

Set a flag after frame decode that prevents `display_update()` from running until
TX completes. This is the simplest change but still relies on the main loop
iterating fast enough — other future blocking operations could reintroduce the bug.

## Verification

After fixing, run `analyze_cycletest.py` on a new capture. Expected:
- **Zero** collision frames (402ms cycles)
- **Zero** timing errors > 0.3ms
- All commands achieve expected HRV state within 2-3 seconds (~10-15 frames)
- Per-command jitter < 0.1ms

## Frame structure reminder

```
[BF][B1][B2][B3][B4][3B][2F][F7][EF][D7]  <gap>  [LCD byte]  <gap>  [BF]...
 B0                                  B9            ^^^^^^^^^^
                                          Fan1: 5.86ms from B9 start (360us from decode)
                                          Fan2: 6.36ms (860us)
                                          Fan3: 6.87ms (1370us)
                                          Fan4: 7.37ms (1870us)
                                          Fan5: 7.88ms (2380us)
                                          Idle: 8.89ms (3390us)
```
