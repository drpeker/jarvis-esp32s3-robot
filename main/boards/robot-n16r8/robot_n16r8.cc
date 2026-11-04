#include "wifi_board.h"

#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_cam_ctlr_dvp.h>
#include <driver/ledc.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <mutex>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/idf_additions.h>
#include <freertos/task.h>

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "application.h"
#include "settings.h"
#include "assets/lang_config.h"
#include "button.h"
#include "codecs/no_audio_codec.h"
#include "config.h"
#include "display/lcd_display.h"
#include "esp32_camera.h"
#include "ina219.h"
#include "mpu6050.h"
#include "net_console.h"
#include "robot_console.h"
#include "mcp_server.h"
#include "battery_gauge.h"
#include "body_sense.h"
#include "gemini_face_display.h"
#include "motor_controller.h"
#include "led/single_led.h"

#define TAG "RobotN16R8"

class RobotN16R8Board : public WifiBoard {
private:
    Button boot_button_;
    LcdDisplay* display_ = nullptr;
    Esp32Camera* camera_ = nullptr;
    MotorController* motor_controller_ = nullptr;
    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    Ina219* ina219_ = nullptr;
    BatteryGauge* battery_gauge_ = nullptr;
    Mpu6050* imu_ = nullptr;
    BodySense* body_sense_ = nullptr;
    enum BatteryState { kBatteryNormal, kBatteryLow, kBatteryCritical };
    std::atomic<int> battery_state_{kBatteryNormal};
    std::atomic<int> battery_average_mv_{0};

    std::string GetWifiSsidPrefix() const override { return "Robot"; }
    std::string GetWifiHostnamePrefix() const override { return "robot"; }

    void InitializeSpi() {
        spi_bus_config_t bus_config = {};
        bus_config.mosi_io_num = DISPLAY_SPI_MOSI_PIN;
        bus_config.miso_io_num = GPIO_NUM_NC;
        bus_config.sclk_io_num = DISPLAY_SPI_SCK_PIN;
        bus_config.quadwp_io_num = GPIO_NUM_NC;
        bus_config.quadhd_io_num = GPIO_NUM_NC;
        bus_config.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(DISPLAY_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO));
    }

    void InitializeDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = DISPLAY_SPI_CLOCK_HZ;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(DISPLAY_SPI_HOST, &io_config, &panel_io));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
        // RES is tied to 3V3 (no hardware reset line), so esp_lcd_panel_reset issues a
        // software SWRESET. The ST7789 needs ~120ms after SWRESET before it accepts
        // commands, but the driver only waits 20ms -- marginal, and the panel stayed
        // black once wake-word/LED boot work shifted the timing. Give it a solid delay.
        vTaskDelay(pdMS_TO_TICKS(150));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
        // The driver sends no gamma, so the panel used its defaults and blacks looked grey
        // (BL is on 3V3, it cannot be dimmed). Gamma of the common 1.69" ST7789V2 module.
        static const uint8_t kGammaPos[] = {0xD0, 0x08, 0x0E, 0x09, 0x09, 0x05, 0x31,
                                            0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34};
        static const uint8_t kGammaNeg[] = {0xD0, 0x08, 0x0E, 0x09, 0x09, 0x15, 0x31,
                                            0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34};
        esp_lcd_panel_io_tx_param(panel_io, 0xE0, kGammaPos, sizeof(kGammaPos));
        esp_lcd_panel_io_tx_param(panel_io, 0xE1, kGammaNeg, sizeof(kGammaNeg));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
        ESP_ERROR_CHECK(
            esp_lcd_panel_set_gap(panel, DISPLAY_PANEL_GAP_X, DISPLAY_PANEL_GAP_Y));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

        display_ = new GeminiFaceDisplay(panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                     DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X,
                                     DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    // GPIO4/5 bus is shared by the camera SCCB and the INA219, so it is created here
    // once and handed to the camera instead of letting esp_video open its own.
    void InitializeI2c() {
        i2c_master_bus_config_t bus_config = {};
        bus_config.i2c_port = static_cast<i2c_port_t>(I2C_BUS_PORT);
        bus_config.sda_io_num = I2C_SDA_PIN;
        bus_config.scl_io_num = I2C_SCL_PIN;
        bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_config.glitch_ignore_cnt = 7;
        bus_config.flags.enable_internal_pullup = 1;
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_));
    }

    // Logs every device that answers on the shared bus (camera 0x30, IMU 0x68/0x69,
    // INA219 0x40..) so wiring problems show up in the boot log.
    void ScanI2cBus() {
        std::string found;
        for (uint8_t address = 0x08; address < 0x78; ++address) {
            if (i2c_master_probe(i2c_bus_, address, 20) == ESP_OK) {
                char text[8];
                snprintf(text, sizeof(text), " 0x%02X", address);
                found += text;
            }
        }
        ESP_LOGI(TAG, "I2C devices on GPIO4/5:%s", found.empty() ? " none" : found.c_str());
    }

    void InitializeImu() {
        const uint8_t address =
            i2c_master_probe(i2c_bus_, IMU_I2C_ADDR, 20) == ESP_OK ? IMU_I2C_ADDR
                                                                    : IMU_I2C_ADDR_ALT;
        imu_ = new Mpu6050(i2c_bus_, address);
        if (!imu_->IsPresent()) {
            delete imu_;
            imu_ = nullptr;
        }
    }

    void InitializePowerSensor() {
        ina219_ = new Ina219(i2c_bus_, INA219_I2C_ADDR, INA219_SHUNT_MILLIOHMS);
        if (ina219_->IsPresent()) {
            battery_gauge_ = new BatteryGauge(ina219_);
            static_cast<GeminiFaceDisplay*>(display_)->SetBatteryProvider(
                [this](int& percent, int& level) {
                    const auto snapshot = battery_gauge_->Get();
                    percent = snapshot.percent;
                    level = snapshot.level;
                    return snapshot.ok;
                });
        }

        McpServer::GetInstance().AddTool(
            "self.robot.self_awareness",
            "Jarvis's awareness of its own body: battery voltage, current draw, power, "
            "estimated battery percent (INA219 sensor), forward/backward speed factor, turn "
            "calibration, whether it is moving, and battery_state (normal/low/critical; at "
            "critical the motors are locked until the battery is charged). Call this whenever "
            "the user asks about Jarvis himself, his battery, voltage, energy, power, whether "
            "he is being held, tilted or bumped (body), or how he feels.",
            PropertyList(), [this](const PropertyList&) -> ReturnValue {
                std::string json = "{\"name\":\"Jarvis\"";
                if (battery_gauge_ != nullptr) {
                    static const char* const kStateNames[] = {"normal", "low", "critical"};
                    json += ",\"battery_protection\":\"" +
                            std::string(kStateNames[battery_state_.load()]) +
                            "\",\"motors_locked_for_battery\":" +
                            (battery_state_.load() == kBatteryCritical ? "true" : "false") +
                            ",\"battery\":" + battery_gauge_->StatusJson();
                } else {
                    json += ",\"power_sensor\":\"not_detected\"";
                }
                if (body_sense_ != nullptr) {
                    json += ",\"body\":" + body_sense_->StatusJson();
                }
                if (motor_controller_ != nullptr) {
                    json += ",\"speed_factor_percent\":" +
                            std::to_string(motor_controller_->SpeedFactor()) +
                            ",\"turn_ms_per_degree\":" +
                            std::to_string(motor_controller_->TurnMsPerDegree()) +
                            ",\"moving\":" + (motor_controller_->IsMoving() ? "true" : "false");
                }
                return json + "}";
            });
    }

    void StartBatteryMonitor() {
        if (battery_gauge_ == nullptr) {
            return;
        }
        // PSRAM stack: internal RAM is needed by TLS, the camera and MCP calls.
        xTaskCreateWithCaps(
            [](void* arg) { static_cast<RobotN16R8Board*>(arg)->BatteryMonitorLoop(); },
            "battery_mon", 3072, this, 2, nullptr, MALLOC_CAP_SPIRAM);
    }

    void BatteryMonitorLoop() {
        int samples[3] = {};
        int sample_count = 0;
        TickType_t last_motion = xTaskGetTickCount();
        TickType_t last_reminder = 0;

        while (true) {
            vTaskDelay(pdMS_TO_TICKS(BATTERY_CHECK_INTERVAL_MS));
            const TickType_t now = xTaskGetTickCount();
            if (motor_controller_->IsMoving()) {
                last_motion = now;
                continue;
            }
            if (now - last_motion < pdMS_TO_TICKS(BATTERY_SETTLE_MS)) {
                continue;
            }
            const auto snapshot = battery_gauge_->Get();
            if (!snapshot.ok) {
                continue;
            }
            samples[sample_count % 3] = static_cast<int>(snapshot.volts * 1000.0f);
            if (++sample_count < 3) {
                continue;
            }
            const int average_mv = (samples[0] + samples[1] + samples[2]) / 3;
            battery_average_mv_.store(average_mv);

            const int state = battery_state_.load();
            int next = state;
            if (average_mv < BATTERY_CRITICAL_MV) {
                next = kBatteryCritical;
            } else if (state == kBatteryCritical) {
                next = average_mv >= BATTERY_RECOVER_MV ? kBatteryNormal : kBatteryCritical;
            } else if (average_mv < BATTERY_LOW_MV) {
                next = kBatteryLow;
            } else if (state == kBatteryLow && average_mv >= BATTERY_RECOVER_MV) {
                next = kBatteryNormal;
            }

            if (next != state) {
                battery_state_.store(next);
                motor_controller_->SetMotionLocked(next == kBatteryCritical);
                if (next == kBatteryNormal) {
                    ESP_LOGI(TAG, "Battery recovered: %d mV, motors unlocked", average_mv);
                } else {
                    ESP_LOGW(TAG, "Battery %s: %d mV%s",
                             next == kBatteryLow ? "LOW" : "CRITICAL", average_mv,
                             next == kBatteryCritical ? ", motors locked" : "");
                    PlayLowBatterySound();
                    last_reminder = now;
                }
            } else if (next == kBatteryCritical &&
                       now - last_reminder >= pdMS_TO_TICKS(BATTERY_CRITICAL_REMIND_MS)) {
                PlayLowBatterySound();
                last_reminder = now;
            }
        }
    }

    static void PlayLowBatterySound() {
        auto& app = Application::GetInstance();
        app.Schedule([&app]() { app.PlaySound(Lang::Sounds::OGG_LOW_BATTERY); });
    }

    void InitializeButton() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    // OV2640 through the esp32-camera driver, photo on demand: the driver starts for each
    // photo (it resets the sensor over SCCB with XCLK already running), takes the JPEG the
    // sensor makes itself and stops again. SCCB uses the shared I2C bus (port 1).
    void InitializeCamera() {
        camera_config_t config = {};
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.pin_xclk = CAMERA_PIN_XCLK;
        config.pin_sccb_sda = -1;
        config.pin_sccb_scl = -1;
        config.sccb_i2c_port = I2C_BUS_PORT;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href = CAMERA_PIN_HREF;
        config.pin_pclk = CAMERA_PIN_PCLK;
        config.xclk_freq_hz = CAMERA_XCLK_FREQUENCY_HZ;
        config.ledc_timer = LEDC_TIMER_0;      // motors use timer 2, channels 1-4
        config.ledc_channel = LEDC_CHANNEL_0;
        config.pixel_format = PIXFORMAT_JPEG;  // the sensor encodes the JPEG itself
        config.frame_size = FRAMESIZE_VGA;     // 640x480
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_LATEST;

        camera_ = new Esp32Camera(config, true);
        if (!camera_->IsReady()) {
            ESP_LOGE(TAG, "Camera initialization failed; camera MCP tool will be disabled");
            delete camera_;
            camera_ = nullptr;
            return;
        }
        // The Sense camera is mounted upside down in the robot's chassis.
        camera_->SetVFlip(true);
        camera_->SetHMirror(true);
    }

public:
    RobotN16R8Board() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeSpi();
        InitializeDisplay();
        InitializeButton();
        InitializeI2c();
        InitializeCamera();
        ScanI2cBus();
        InitializeImu();
        motor_controller_ = new MotorController(MOTOR_LEFT_IN1_PIN, MOTOR_LEFT_IN2_PIN,
                                                MOTOR_RIGHT_IN1_PIN, MOTOR_RIGHT_IN2_PIN);
        motor_controller_->SetGyro(imu_);
        InitializePowerSensor();
        StartBatteryMonitor();
        if (imu_ != nullptr) {
            body_sense_ = new BodySense(imu_, motor_controller_);
        }
        NetConsole::GetInstance().Start(23);
        RegisterRobotConsole(motor_controller_, imu_, battery_gauge_);
        NetConsole::GetInstance().AddCommand(
            "photo", "photo [drop_frames]  - test photo through the AI photo path (result and time)",
            [this](const NetConsole::Args& args) {
                if (camera_ == nullptr) {
                    NetConsole::GetInstance().Print("camera not available\r\n");
                    return;
                }
                std::lock_guard<std::mutex> lock(*camera_->GetMutex());
                camera_->SetDropFrames(args.size() > 1 ? std::atoi(args[1].c_str()) : 3);
                const int64_t t0 = esp_timer_get_time();
                const bool ok = camera_->Capture();
                camera_->SetDropFrames(3);
                NetConsole::GetInstance().Print("photo %s in %d ms, internal RAM free %u (min %u)\r\n",
                                                ok ? "OK" : "FAILED",
                                                static_cast<int>((esp_timer_get_time() - t0) / 1000),
                                                heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                                                heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
            });
        ESP_LOGI(TAG, "Robot (N16R8) board initialized");
    }

    virtual AudioCodec* GetAudioCodec() override {
        // INMP441 (I2S std): SCK/WS/DIN.  MAX98357A: BCLK/LRCK/DOUT.
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                                               AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK,
                                               AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK,
                                               AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override { return display_; }

    virtual Camera* GetCamera() override { return camera_; }

    // NOTE (2026-11-01): WS2812 GetLed on GPIO48 TEMPORARILY DISABLED while isolating
    // the black-screen bug. It is the only early-boot change since the known-good build
    // #1 (wake word loads at ~20s, so it is ruled out). If the screen returns with this
    // removed, the RMT/WS2812 on GPIO48 is the culprit and we re-add the LED safely.
    // virtual Led* GetLed() override {
    //     static SingleLed led(GPIO_NUM_48);
    //     return &led;
    // }
};

DECLARE_BOARD(RobotN16R8Board);
