# Mochan V2

Mochan V2 is a two-wheel AI robot based on the Seeed Studio XIAO ESP32S3
Sense. It combines the onboard OV3660 camera and PDM microphone with a
240x240 ST7789 display, a MAX98357A I2S amplifier, and an MX1508 dual motor
driver.

## Wiring

### ST7789 240x240

| ST7789 | XIAO pin | ESP32-S3 GPIO |
| --- | --- | --- |
| GND | GND | - |
| VCC | 3V3 | - |
| SCL | D10 | GPIO9 |
| SDA | D9 | GPIO8 |
| RST | D8 | GPIO7 |
| DC | D7 | GPIO44 |
| CS | GND | - |
| BL | 3V3 | - |

### MAX98357A

| MAX98357A | XIAO pin | ESP32-S3 GPIO |
| --- | --- | --- |
| VIN | LiPo positive (3.0-4.2V) | - |
| GND | Common GND | - |
| DIN | D4 | GPIO5 |
| BCLK | D5 | GPIO6 |
| LRC | D6 | GPIO43 |
| SD | Leave open | - |
| GAIN | Leave open | - |

Connect the 4-ohm 3W speaker only between the amplifier's `+` and `-` output
pads. Neither speaker terminal is ground.

### MX1508

| MX1508 | XIAO pin | ESP32-S3 GPIO |
| --- | --- | --- |
| + | LiPo positive (3.0-4.2V) | - |
| - | Common GND | - |
| IN1 | D0 | GPIO1 (right motor) |
| IN2 | D1 | GPIO2 (right motor) |
| IN3 | D2 | GPIO3 (left motor) |
| IN4 | D3 | GPIO4 (left motor) |
| MOTOR-A | Right motor | - |
| MOTOR-B | Left motor | - |

The installed motors support 3-12V, so `MOTOR_MAX_DUTY_PERCENT` is set to
100% in `config.h`. `MOTOR_MIN_DUTY_PERCENT` is set to 100%, so every
non-zero motor command runs at full power.

## Power

Use a protected 1S LiPo battery and a star connection:

```text
LiPo + ---- XIAO BAT+
       +--- MAX98357A VIN
       +--- MX1508 +

LiPo - ---- XIAO BAT-
       +--- MAX98357A GND
       +--- MX1508 -
```

Power the TFT from the XIAO `3V3` output. Add at least 470uF across the MX1508
power input, 220uF across MAX98357A VIN/GND, and 100nF directly across each
motor terminal pair. Keep motor wiring away from the microphone and camera
flex cable.

## MCP tools

- `self.wall_e.drive(direction, speed, duration_ms)`
- `self.wall_e.run_motor(motor, direction, speed, duration_ms)`
- `self.wall_e.stop()`
- `self.wall_e.get_status()`

Every movement has a maximum duration of five seconds. The motor task stops
both motors automatically when the timeout expires. A 120ms coast interval is
inserted before polarity reversal, and the two motors start 40ms apart to reduce
current spikes while preserving 100% running power.

## Mochan face UI

The custom display uses the GIF files from the project-level `eye` directory:

- the upper 240x100 area shows an animated eye expression;
- the lower 240x140 area shows UTF-8 conversation text with a typewriter effect;
- neutral expressions randomly alternate between the normal and four looking directions;
- Xiaozhi LLM emotions select happy, sad, angry, suspicious, or shaking eyes;
- a camera snapshot temporarily replaces only the lower text area for five seconds, while the
  animated eyes and status strip remain visible.

Keep replacement eye files at exactly 240x100 pixels and retain their current
filenames so the emotion mapping continues to work.

## Build

This Mochan V2 workspace is verified with ESP-IDF 5.5.2:

```bash
source /path/to/esp-idf/export.sh
python3 scripts/release.py mochan-v2
```

## Firmware updates

Remote firmware and asset updates are disabled for this board. The 8MB flash
uses one large `factory` application partition and a separate local assets
partition. Flash `merged-binary.bin` at offset `0x0` over USB whenever the
firmware or eye assets change. Switching from an older OTA build to this layout
requires this full USB flash once; flashing only `xiaozhi.bin` is not sufficient.

The hardware must be bench-tested with the wheels lifted before placing the
robot on the floor. If forward/backward or left/right is reversed, adjust
`MOTOR_LEFT_INVERT` and `MOTOR_RIGHT_INVERT` in `config.h` or swap the two
wires of the affected motor.
