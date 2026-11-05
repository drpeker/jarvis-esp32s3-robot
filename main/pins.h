// Jarvis hardware: ESP32-S3 WROOM N16R8 robot wiring.
#pragma once
#include <driver/gpio.h>

// INMP441 microphone (I2S0 RX, L/R to GND -> left channel), 16 kHz.
constexpr gpio_num_t PIN_MIC_SCK = GPIO_NUM_41;
constexpr gpio_num_t PIN_MIC_WS = GPIO_NUM_42;
constexpr gpio_num_t PIN_MIC_DIN = GPIO_NUM_46;
constexpr int MIC_SAMPLE_RATE = 16000;

// MAX98357A amplifier (I2S1 TX).
constexpr gpio_num_t PIN_SPK_BCLK = GPIO_NUM_47;
constexpr gpio_num_t PIN_SPK_LRCK = GPIO_NUM_3;
constexpr gpio_num_t PIN_SPK_DOUT = GPIO_NUM_45;

// ST7789 240x240 (SPI2, mode 3, CS to GND, backlight on 3V3).
constexpr gpio_num_t PIN_LCD_SCK = GPIO_NUM_39;
constexpr gpio_num_t PIN_LCD_MOSI = GPIO_NUM_40;
constexpr gpio_num_t PIN_LCD_DC = GPIO_NUM_38;
constexpr gpio_num_t PIN_LCD_RST = GPIO_NUM_48;
constexpr int LCD_WIDTH = 240;
constexpr int LCD_HEIGHT = 240;
constexpr int LCD_SPI_HZ = 20 * 1000 * 1000;

// MX1508 dual H-bridge.
constexpr gpio_num_t PIN_MOTOR_L_IN1 = GPIO_NUM_14;
constexpr gpio_num_t PIN_MOTOR_L_IN2 = GPIO_NUM_21;
constexpr gpio_num_t PIN_MOTOR_R_IN1 = GPIO_NUM_1;
constexpr gpio_num_t PIN_MOTOR_R_IN2 = GPIO_NUM_2;

// Shared I2C bus (port 1): OV2640 SCCB 0x30, INA219 0x40, MPU-6050 0x68.
constexpr int I2C_PORT = 1;
constexpr gpio_num_t PIN_I2C_SDA = GPIO_NUM_4;
constexpr gpio_num_t PIN_I2C_SCL = GPIO_NUM_5;
constexpr uint32_t I2C_HZ = 100000;
constexpr uint8_t INA219_ADDR = 0x40;
constexpr uint8_t MPU6050_ADDR = 0x68;
constexpr uint8_t OV2640_ADDR = 0x30;

// OV2640 DVP camera (no RESET / PWDN wired; mounted upside down).
constexpr int PIN_CAM_XCLK = 15;
constexpr int PIN_CAM_D0 = 11, PIN_CAM_D1 = 9, PIN_CAM_D2 = 8, PIN_CAM_D3 = 10;
constexpr int PIN_CAM_D4 = 12, PIN_CAM_D5 = 18, PIN_CAM_D6 = 17, PIN_CAM_D7 = 16;
constexpr int PIN_CAM_VSYNC = 6, PIN_CAM_HREF = 7, PIN_CAM_PCLK = 13;

constexpr gpio_num_t PIN_BOOT_BUTTON = GPIO_NUM_0;
