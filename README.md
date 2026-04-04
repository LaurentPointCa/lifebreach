# LifeBreach

ESP32 controller for the LifeBreath RNC6-ES Heat Recovery Ventilator (HRV). Replaces/augments the DXPL02 wall controller with WiFi connectivity, Home Assistant integration, and full bidirectional control.

## What it does

- **Monitors** the HRV's 2000 baud serial bus, decoding mode (Fresh/Recirc) and fan speed (0-5)
- **Controls** the HRV by injecting timing-encoded command bytes into the bus
- **Relays** the HRV data stream to the original LCD wall panel via a second UART
- **Publishes** state to Home Assistant via MQTT with auto-discovery
- **Displays** live status on a 128x64 SSD1306 OLED

## Hardware

| Component | Details |
|-----------|---------|
| MCU | DOIT ESP32 DEVKIT V1 (WROOM, dual-core, USB-C) |
| Display | SSD1306 128x64 OLED (I2C, yellow/blue split) |
| Isolation | 4x PC817 optocouplers (galvanic isolation for HRV and LCD buses) |
| HRV | LifeBreath RNC6-ES (3-wire proprietary serial bus) |
| Wall panel | DXPL02 (nRF51422-based LCD controller) |

### Bus wiring

The HRV uses a 3-wire bus: RED (+4.5V power), YEL (data), GRN (ground). The data line carries active-low 2000 baud UART — the HRV broadcasts 10-byte status frames, and the controller sends 1 command byte per cycle.

The ESP32 sits between the HRV and LCD panel as a man-in-the-middle:

```
HRV bus ──opto 1/2──> ESP32 UART2 (GPIO16 RX / GPIO17 TX)
LCD bus ──opto 3/4──> ESP32 UART1 (GPIO5 RX  / GPIO4 TX)
OLED ─────────────────ESP32 I2C   (GPIO21 SDA / GPIO22 SCL)
```

Each optocoupler provides full galvanic isolation between the 4.5V bus and the ESP32's 3.3V logic. RX optos include a 1N4007 diode in series to prevent partial conduction at the bus LOW voltage (1.32V).

## Protocol

The HRV protocol was fully reverse-engineered from logic analyzer captures.

### Frame structure (HRV to controller, every ~200ms)

```
[0xBF][B1][B2][B3][B4][0x3B][0x2F][0xF7][0xEF][0xD7]
  sync                                            D7
```

- **B3**: Mode — `0x5E` = Fresh, `0x5D` = Recirc
- **B1, B2, B3**: Combined lookup determines fan speed (0-5)
- Bytes are spaced 20.094ms apart (start-to-start)

### Command bytes (controller to HRV)

The controller sends one byte per cycle in the gap after the HRV frame. Both the byte value AND its arrival timing encode the command:

| Command | Byte (Fresh/Recirc) | Delay from D7 |
|---------|---------------------|---------------|
| Fan 1 | 0xDF / 0xCF | 5.86ms |
| Fan 2 | 0xEF / 0xE7 | 6.36ms |
| Fan 3 | 0xF7 / 0xF3 | 6.87ms |
| Fan 4 | 0xFB / 0xF9 | 7.37ms |
| Fan 5 | 0xFD / 0xFC | 7.88ms |
| Idle | 0xFF | 8.89ms |

Timing slots are spaced ~500us apart (1 bit period at 2000 baud). The HRV validates both byte value and arrival time — a byte at the wrong delay is ignored or misinterpreted.

### Bathroom timer

A 7ms active-low pulse on the data line (not UART-framed) triggers high-speed bypass mode. Detected via UART break detection.

## Software architecture

```
lifebreach.ino  ── OLED display, buttons, setup/loop
hrv_uart.cpp/h  ── UART init, frame decode, TX timing, FreeRTOS task, LCD relay
wifi_mqtt.cpp/h ── WiFi (captive portal), MQTT, Home Assistant auto-discovery, OTA
hrv_protocol.h  ── Protocol constants, lookup tables, helper functions
config.h        ── MQTT topics, WiFi/OTA configuration
```

### Timing-critical design

The TX command byte must arrive within ~250us of the correct timing slot. To guarantee this:

- **FreeRTOS task** (`hrv_task`, priority 19, core 1) handles all UART RX/TX, isolated from OLED I2C writes (15-20ms blocking) and WiFi/MQTT on the main loop
- **Spin-wait** on `micros()` for precise TX delay after frame decode
- **Hardware timer ISR** sends LCD relay bytes every 20.094ms with microsecond precision, matching the HRV's native clock rate

### Home Assistant integration

- **WiFiManager** captive portal for credential setup (SSID, MQTT broker IP/user/pass)
- **MQTT** state publishing (mode, fan, uptime, RSSI) on change + 30s heartbeat
- **HA auto-discovery** — sensors and a select entity appear automatically
- **ArduinoOTA** for wireless firmware updates

## Building

1. Install [Arduino IDE](https://www.arduino.cc/en/software) with ESP32 board package
2. Select board: **DOIT ESP32 DEVKIT V1**
3. Install libraries via Library Manager:
   - **WiFiManager** by tzapu
   - **PubSubClient** by Nick O'Leary
   - **Adafruit SSD1306** + **Adafruit GFX**
4. Open `lifebreach.ino` and upload

## First boot

1. ESP32 creates WiFi AP: **LifeBreach-Setup**
2. Connect with phone, configure WiFi + MQTT broker credentials
3. After connecting, entities appear in Home Assistant under **LifeBreach HRV**
4. OLED shows live HRV mode, fan speed, and connection status

## License

MIT
