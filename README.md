# JK-BMS CYD Monitor

ESP32 Cheap Yellow Display firmware for Jikong (JK) BMS monitoring.

Reads battery data (cell voltages, SOC, current, temperatures) via Bluetooth Low Energy, serves a JSON API over WiFi, and displays live battery status on the CYD touchscreen.

## Features

- **BLE reading** — Connects to JK-BMS (v11/v15/v19) via Bluetooth
- **CYD display** — 2.4" TFT shows SOC bar, cell voltages, temperatures, stats
- **Touch controls** — Tap buttons to toggle charge/discharge/balance
- **Web dashboard** — Full-featured dashboard accessible via browser
- **JSON API** — `/data` endpoint for external monitoring
- **Multi-cell support** — Up to 16 cells (default 4S)

## Hardware

- ESP32 Cheap Yellow Display (CYD) — 2.4" ILI9341 TFT + XPT2046 touch
- Jikong (JK) BMS — any model with BLE (JK_B2A20S4P, JK_BD4A20S4P, etc.)
- USB cable for flashing

## Libraries Required

Install via Arduino IDE Library Manager:

| Library | Version |
|---------|---------|
| TFT_eSPI | Latest |
| XPT2046_Touchscreen | Latest |
| ArduinoJson | 7.x |

## Setup

### 1. Configure TFT_eSPI

Copy the CYD pin config to your TFT_eSPI library:

```bash
cp TFT_eSPI_User_Setup_CYD.h ~/.arduino15/packages/esp32/hardware/esp32/<version>/libraries/TFT_eSPI/User_Setup.h
```

Or place as `User_Setups/Setup44_CYD.h` and uncomment `#include <User_Setups/Setup44_CYD.h>` in `User_Setup.h`.

### 2. Configure the Sketch

In `jk_bms_cyd.ino`, edit these lines:

```cpp
static const char WiFi_SSID[]    = "YOUR_WIFI_SSID";
static const char WiFi_Password[] = "YOUR_WIFI_PASSWORD";
static const char BMS_MAC[]      = "20:22:08:25:26:8b";  // Your BMS MAC
```

Find your BMS MAC address:
- Open the JK-BMS phone app and check the device info
- Or compile/upload the sketch, watch the serial output — it will print discovered BLE devices

### 3. Upload

1. Open `jk_bms_cyd.ino` in Arduino IDE
2. Select your board (ESP32 Dev Module)
3. Set partitions to **Default 4MB**
4. Upload

### 4. Upload Web Files

After upload:
1. Open Serial Monitor (115200 baud)
2. Note the ESP32 IP address
3. Visit `http://<IP>/fs` in your browser
4. Upload `data/index.html` and `data/style.css`

### 5. View Dashboard

Visit `http://<IP>` — you'll see:
- Live battery voltage, current, power, SOC
- Per-cell voltage readings
- Temperature readings (T1, T2, MOS)
- Toggle buttons for charge/discharge/balance

## Touch Screen Controls

The CYD display shows 3 buttons at the bottom:

| Button | Action | Color |
|--------|--------|-------|
| **CHARGE** | Toggle charging | Green when ON |
| **DISCHG** | Toggle discharging | Blue when ON |
| **BALANCE** | Toggle balancing | Orange when ON |

## Web Dashboard

### Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | HTML dashboard |
| `/data` | GET | JSON data (auto-refresh) |
| `/control` | POST | Toggle BMS functions |
| `/freeheap` | GET | Free memory info |
| `/uptime` | GET | Uptime info |
| `/fs` | GET | File manager |

### Control API

```bash
curl -X POST http://<IP>/control \
  -H "Content-Type: application/json" \
  -d '{"action":"charging","state":"on"}'

curl -X POST http://<IP>/control \
  -H "Content-Type: application/json" \
  -d '{"action":"discharging","state":"off"}'

curl -X POST http://<IP>/control \
  -H "Content-Type: application/json" \
  -d '{"action":"balance","state":"on"}'
```

Actions: `charging`, `discharging`, `balance`
States: `on`, `off`

### JSON Data Format

```json
{
  "battery_voltage": 16.64,
  "battery_power": 0.0,
  "charge_current": 0.0,
  "percent_remain": 77,
  "capacity_remain": 30.81,
  "nominal_capacity": 40.00,
  "cycle_count": 0,
  "cell_voltages": [3.695, 3.696, 3.696, 3.698],
  "avg_cell_voltage": 3.70,
  "delta_cell_voltage": 0.01,
  "battery_t1": 22.0,
  "battery_t2": 22.9,
  "mos_temp": 28.1,
  "charge": true,
  "discharge": true,
  "balance": false,
  "balancing_action": 0
}
```

## Troubleshooting

### Display shows "NO WIFI"
- Check WiFi credentials in the sketch
- Ensure ESP32 is within range of your router

### Display shows "SCANNING..."
- ESP32 can't find the BMS. Verify the MAC address matches your BMS.
- Make sure the BMS is powered on and BLE is active.
- Distance: keep the CYD within ~3m of the BMS during initial connection.

### Touch not responding
- Touch calibration may need adjustment. Modify `TOUCH_MIN_X`, `TOUCH_MAX_X`, `TOUCH_MIN_Y`, `TOUCH_MAX_Y` in the sketch.

### BMS disconnects frequently
- The BMS may be going to sleep. Move it closer to the ESP32.
- Increase BLE power: change `NimBLEDevice::setPower(3)` to `5` (max).

### Web dashboard won't load
- Ensure you uploaded `index.html` and `style.css` to LittleFS via `/fs`

## References

- Based on: https://github.com/peff74/Arduino-jk-bms
- CYD pinout: https://github.com/witnessmenow/ESP32-Cheap-Yellow-Display
- NimBLE-Arduino: https://github.com/h2zero/NimBLE-Arduino