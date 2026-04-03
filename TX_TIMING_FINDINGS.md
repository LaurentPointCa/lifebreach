# TX Timing Findings — ESP32 vs Real LCD (DXPL02)

## Problem

The ESP32 sends the correct command byte in the correct inter-frame slot with excellent
precision, but the HRV ignores it. Analysis of multiple reference captures reveals the
real LCD sends **command bytes significantly earlier** than idle bytes. The ESP32 was
calibrated to idle (0xFF) timing and used that same timing for commands.

## Key Discovery: Command bytes arrive earlier than idle bytes

Measured across ALL reference captures with the real DXPL02 LCD:

| LCD byte | Delay from D7 start | Delay from frame decode |
|----------|---------------------|-------------------------|
| Command (0xCF, 0xDF, etc.) | **5.86ms** | **~860us** |
| Idle (0xFF) | **8.38–8.89ms** | **~3400–3900us** |

Sources confirming 5.86ms for commands:
- `hrv-idle-connectlcd-fan1.vcd` — LCD hot-connect, 0xDF at 5.86ms, **HRV responds**
- `hrv-lcd-mode-recirc-fancycle.vcd` — 0xCF at 5.86ms, **HRV responds**
- `hrv-lcd-mode-fresh-fancycle.vcd` — 0xDF at 5.86ms, **HRV responds**
- `hrv-powerup-with-devices2.vcd` — 0xCF at 5.86ms during powerup, **HRV responds**

The ESP32 (boot5) sent 0xCF at **8.37ms** — the idle timing. The HRV ignored it.

## ESP32 Capture History

| Capture | TX delay from D7 | Result |
|---------|-------------------|--------|
| boot2 | 9.7ms (millis, sloppy) | Ignored |
| boot3 | 13.65ms (wrong constant) | Ignored |
| boot4 | 8.65ms (close to idle timing) | Ignored |
| boot5 | 8.37ms (nailed idle timing) | **Ignored** |

Boot5 had perfect precision matching the real LCD's idle (0xFF) timing —
but the HRV ignores command bytes sent at idle timing.

## Required Fix

### Change TX_DELAY_US for command bytes

```
TX_DELAY_US = 860    // microseconds from frame decode to TX byte start
                     // yields ~5.86ms from D7 start on wire
                     // (matches real LCD command byte timing)
```

**Why 860:** Frame decode fires ~5.0ms after D7 start (after the full byte is
received). Target is 5.86ms from D7 start. So: 5860 - 5000 = 860us from frame decode.

For idle (0xFF) the old timing (~3400us) was correct, but the HRV doesn't need
idle bytes to be precisely timed — it only matters for commands. For simplicity,
using 860us for all bytes (including idle) is fine as a first test.

### Expected result on wire
```
D7 start ──5.0ms──> frame decoded ──0.86ms──> TX byte

Total: 5.0 + 0.86 = 5.86ms from D7 start  ✓ (matches real LCD commands)
```

### Key constant
```
TX_DELAY_US      860     // microseconds from frame decode to TX byte start
                         // yields ~5.86ms from D7 start on wire
```

## Verification

After fixing, capture with the logic analyzer and run `analyze_timing.py` from the
project root. The "LCD byte delay" column should show ~5.86ms. If the HRV responds,
its frame bytes will change within ~2 seconds (10 frames) of the first command.

## Frame structure reminder

```
[BF][B1][B2][B3][B4][3B][2F][F7][EF][D7]  <gap>  [LCD byte]  <gap>  [BF]...
 B0                                  B9            ^^^^^^^^^^
                                                   Target: 5.86ms from B9 start
                                                   = ~860us from frame decode
```
