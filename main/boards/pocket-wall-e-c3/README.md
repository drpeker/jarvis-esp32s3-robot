# Pocket Wall-E ESP32-C3 SuperMini

Compact Xiaozhi voice assistant for a 128x64 SSD1306 OLED, INMP441 microphone,
MAX98357A amplifier, and one external action button.

## Wiring

| Function | GPIO |
| --- | ---: |
| OLED SDA | 21 |
| OLED SCL | 20 |
| I2S BCLK/SCK (shared) | 2 |
| I2S WS/LRC (shared) | 1 |
| MAX98357A DIN | 3 |
| INMP441 SD | 8 |
| Action button to GND | 10 |

The firmware targets common 4 MB ESP32-C3 SuperMini boards. It uses a single
factory application partition, the English UI, and the lightweight WakeNet9s
`Hi ESP` model. GPIO10 provides push-to-talk/toggle-chat control and Wi-Fi
configuration access. The INMP441 L/R pin is tied to GND. The OLED and
microphone are powered from 3.3 V; the MAX98357A is powered from 5 V. All
modules must share GND.

The OLED face is an original LVGL implementation inspired by the appearance and
documented behaviors of [FluxGarage RoboEyes](https://github.com/FluxGarage/RoboEyes),
including eased movement, auto blinking, idle glances, mood eyelids, and small
laugh/confused motion cues. No Adafruit GFX or RoboEyes source is embedded.
