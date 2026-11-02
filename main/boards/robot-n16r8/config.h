#ifndef _ROBOT_N16R8_CONFIG_H_
#define _ROBOT_N16R8_CONFIG_H_

#include <driver/gpio.h>
#include <driver/spi_master.h>

// ESP32-S3 WROOM N16R8 (16MB flash / 8MB OCT PSRAM), dual USB-C, camera expansion.
// Pin plan matches the NeuroVision wiring: ST7789 color LCD, INMP441 I2S mic,
// MAX98357A amp, MX1508 dual H-bridge, OV2640 DVP camera.

// Audio: INMP441 (I2S standard) microphone + external MAX98357A amplifier.
#define AUDIO_INPUT_SAMPLE_RATE 16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

// INMP441 microphone (L/R tied to GND -> left channel).
#define AUDIO_I2S_MIC_GPIO_SCK GPIO_NUM_41
#define AUDIO_I2S_MIC_GPIO_WS GPIO_NUM_42
#define AUDIO_I2S_MIC_GPIO_DIN GPIO_NUM_46

// MAX98357A amplifier.
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_47
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_3
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_45

// Onboard BOOT button.
#define BOOT_BUTTON_GPIO GPIO_NUM_0

// ST7789 240x280 color LCD. CS wired to GND, BL to 3V3. RST now driven by GPIO48
// (hardware reset -- the WS2812 LED was given up for this pin) so the driver does a
// proper timed hardware reset instead of the marginal software SWRESET.
#define DISPLAY_SPI_HOST SPI2_HOST
#define DISPLAY_SPI_SCK_PIN GPIO_NUM_39
#define DISPLAY_SPI_MOSI_PIN GPIO_NUM_40
#define DISPLAY_SPI_CLOCK_HZ (20 * 1000 * 1000)
#define DISPLAY_SPI_MODE 3
#define DISPLAY_RST_PIN GPIO_NUM_48
#define DISPLAY_DC_PIN GPIO_NUM_38
#define DISPLAY_CS_PIN GPIO_NUM_NC
#define DISPLAY_WIDTH 240
#define DISPLAY_HEIGHT 240
#define DISPLAY_PANEL_GAP_X 0
#define DISPLAY_PANEL_GAP_Y 0
#define DISPLAY_OFFSET_X 0
#define DISPLAY_OFFSET_Y 0
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR true

// MX1508 dual H-bridge.
#define MOTOR_LEFT_IN1_PIN GPIO_NUM_14
#define MOTOR_LEFT_IN2_PIN GPIO_NUM_21
#define MOTOR_RIGHT_IN1_PIN GPIO_NUM_1
#define MOTOR_RIGHT_IN2_PIN GPIO_NUM_2
#define MOTOR_LEFT_INVERT false
#define MOTOR_RIGHT_INVERT false
#define MOTOR_PWM_FREQUENCY_HZ 10000
// Lowest non-zero PWM duty; below this the gear motors stall instead of turning.
#define MOTOR_MIN_DUTY_PERCENT 25
#define MOTOR_MAX_DUTY_PERCENT 100
#define MOTOR_MAX_RUN_TIME_MS 5000
#define MOTOR_DIRECTION_DEAD_TIME_MS 120
#define MOTOR_DUAL_START_STAGGER_MS 40
// Starting from standstill below 100% duty, pulse full power this long to break
// static friction, then drop to the requested duty.
#define MOTOR_KICK_START_MS 60

// Speed factor (percent of full PWM) for forward/backward. Changed by voice through
// self.robot.set_speed_factor and kept in NVS.
#define MOTOR_SPEED_FACTOR_DEFAULT 50
#define MOTOR_SPEED_FACTOR_MIN 25
#define MOTOR_SPEED_FACTOR_MAX 100

// Turns run in place at a fixed power, independent of the speed factor. With the IMU
// the angle is measured by the gyro; without it the turn falls back to timing:
// duration = degrees * ms_per_degree (calibrate with self.robot.calibrate_turn).
#define MOTOR_TURN_SPEED_PERCENT 70
#define MOTOR_TURN_DEFAULT_DEGREES 20
#define MOTOR_TURN_MS_PER_DEGREE_DEFAULT 6
#define MOTOR_TURN_MS_PER_DEGREE_MAX 40
// With the IMU, turns are closed-loop on the gyro Z rate: motors stop LEAD degrees
// before the target (coast/brake covers the rest), then brake for BRAKE_MS.
#define MOTOR_TURN_STOP_LEAD_DEGREES 2
#define MOTOR_TURN_BRAKE_MS 80
#define MOTOR_TURN_SAMPLE_MS 5
#define MOTOR_TURN_SETTLE_MS 300

// Shared I2C bus (port 1): OV2640 SCCB + INA219. SDA=GPIO4, SCL=GPIO5.
#define I2C_BUS_PORT 1
#define I2C_SDA_PIN GPIO_NUM_4
#define I2C_SCL_PIN GPIO_NUM_5
#define I2C_BUS_FREQ_HZ 100000

// INA219 voltage/current sensor (A0=A1=GND -> 0x40), standard module shunt R100.
#define INA219_I2C_ADDR 0x40
// MPU-6050/6500 IMU (AD0=GND -> 0x68, AD0=VCC -> 0x69).
#define IMU_I2C_ADDR 0x68
#define IMU_I2C_ADDR_ALT 0x69
#define INA219_SHUNT_MILLIOHMS 100
// Battery: 1S Li-ion 5600mAh.
// Battery gauge (ported from NeuroVision-Gemini): percent counted from the current
// drawn out of the capacity, blended toward the voltage curve while idle.
#define BATTERY_CAPACITY_MAH 5600
#define BATTERY_CELLS 1
#define BATTERY_FULL_CELL_MV 4150
#define BATTERY_IDLE_MA 600
#define BATTERY_BLEND_S 3600.0f
#define BATTERY_POLL_MS 250
// Battery protection (no BMS on the cell). Decisions use the average of the last 3
// readings taken while the motors have been stopped for BATTERY_SETTLE_MS, so motor
// inrush sag does not trigger it. LOW: warning sound. CRITICAL: motors locked
// (stop still works). Unlocks again once the cell is back above RECOVER (charged).
#define BATTERY_LOW_MV 3500
#define BATTERY_CRITICAL_MV 3300
#define BATTERY_RECOVER_MV 3600
#define BATTERY_CHECK_INTERVAL_MS 5000
#define BATTERY_SETTLE_MS 2000
#define BATTERY_CRITICAL_REMIND_MS 60000

// OV2640 DVP camera.
#define CAMERA_PIN_PWDN GPIO_NUM_NC
#define CAMERA_PIN_RESET GPIO_NUM_NC
#define CAMERA_PIN_XCLK GPIO_NUM_15
#define CAMERA_PIN_SIOD I2C_SDA_PIN
#define CAMERA_PIN_SIOC I2C_SCL_PIN
#define CAMERA_PIN_D7 GPIO_NUM_16
#define CAMERA_PIN_D6 GPIO_NUM_17
#define CAMERA_PIN_D5 GPIO_NUM_18
#define CAMERA_PIN_D4 GPIO_NUM_12
#define CAMERA_PIN_D3 GPIO_NUM_10
#define CAMERA_PIN_D2 GPIO_NUM_8
#define CAMERA_PIN_D1 GPIO_NUM_9
#define CAMERA_PIN_D0 GPIO_NUM_11
#define CAMERA_PIN_VSYNC GPIO_NUM_6
#define CAMERA_PIN_HREF GPIO_NUM_7
#define CAMERA_PIN_PCLK GPIO_NUM_13
#define CAMERA_XCLK_FREQUENCY_HZ 20000000

#endif  // _ROBOT_N16R8_CONFIG_H_
