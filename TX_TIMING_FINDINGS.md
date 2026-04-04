# TX Timing Findings — ESP32 vs Real LCD (DXPL02)

## Status: FULLY RESOLVED (2026-04-03)

ESP32 successfully controls all HRV modes and fan speeds. Full cycle test passed:
Fresh F1-5, Recirc F1-5, idle — all recognized by HRV within ~2 seconds.

## Protocol: Timing encodes fan speed

Every command has a unique, precise delay from D7 (last frame byte) start:

| LCD byte | Command | Delay from D7 start | ESP32 TX_DELAY_US |
|----------|---------|---------------------|-------------------|
| 0xCF/0xDF | Fan 1 | **5.86ms** | **560** |
| 0xE7/0xEF | Fan 2 | **6.36ms** | **1060** |
| 0xF3/0xF7 | Fan 3 | **6.87ms** | **1570** |
| 0xF9/0xFB | Fan 4 | **7.37ms** | **2070** |
| 0xFC/0xFD | Fan 5 | **7.88ms** | **2580** |
| 0xFF | Idle | **8.89ms** | **3590** |

Pattern: ~500us (1 bit period at 2000 baud) step per speed level.
Mode (Fresh/Recirc) is encoded in byte value only. Fan speed is encoded in
BOTH byte value AND arrival timing — both must agree.

## Final working implementation

```c
// FINAL WORKING VALUES — hardware timer ISR
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
```

TX byte MUST be sent from a **hardware timer ISR** — main loop polling is not
reliable due to OLED I2C writes blocking for 15-20ms.

## Calibration history

The TX_DELAY_US values went through several rounds of calibration:

| Round | Values (Fan1..Idle) | Issue |
|-------|---------------------|-------|
| 1 | 860..3890 | ~500us too high (UART/ISR latency not accounted for) |
| 2 | 360..3390 | Correct for main-loop polling, but OLED jitter caused collisions |
| 3 | 360..3390 via ISR | ISR fires ~200us earlier than loop polling → all bytes 200us early |
| **4 (final)** | **560..3590 via ISR** | **Correct. All modes/speeds verified.** |

## Frame structure

```
[BF][B1][B2][B3][B4][3B][2F][F7][EF][D7]  <gap>  [LCD byte]  <gap>  [BF]...
 B0                                  B9            ^^^^^^^^^^
                                          Fan1: 5.86ms from B9 start
                                          Fan2: 6.36ms
                                          Fan3: 6.87ms
                                          Fan4: 7.37ms
                                          Fan5: 7.88ms
                                          Idle: 8.89ms
```
