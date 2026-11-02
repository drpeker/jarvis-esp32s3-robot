# Robot N16R8 — "Jarvis" board

ESP32-S3 WROOM-1 N16R8 tracked robot. Full documentation (build, OTA, console, calibration,
tips): see the [project README](../../../README.md).

## Pin plan (from `config.h`)

| Function | Signal | GPIO |
| --- | --- | --- |
| LCD ST7789 240×240 | SCK / MOSI / DC / **RST** | 39 / 40 / 38 / **48** |
| | CS / BL | GND / 3V3 |
| Mic INMP441 | SCK / WS / SD | 41 / 42 / 46 (L/R → GND) |
| Amp MAX98357A | BCLK / LRC / DIN | 47 / 3 / 45 |
| Motors MX1508 | L IN1 / L IN2 / R IN1 / R IN2 | 14 / 21 / 1 / 2 |
| Camera OV2640 | XCLK / PCLK / VSYNC / HREF | 15 / 13 / 6 / 7 |
| | D0..D7 | 11, 9, 8, 10, 12, 18, 17, 16 |
| | PWDN / RESET | not connected |
| **Shared I2C** (port 1, 100 kHz) | **SDA / SCL** | **4 / 5** |
| | OV2640 SCCB | 0x30 |
| | INA219 (VIN+ ← battery +, VIN− → load) | 0x40 |
| | MPU-6050 (AD0 → GND, X forward, Y left, Z up) | 0x68 |
| BOOT button | | 0 |

Add 4.7 kΩ pull-ups from SDA and SCL to 3V3.
