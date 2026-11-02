#include "wifi_board.h"

#include <algorithm>
#include <cstring>
#include <string>

#include <driver/i2c_master.h>
#include <esp_camera.h>
#include <esp_log.h>

#include "application.h"
#include "audio/audio_service.h"
#include "button.h"
#include "codecs/no_audio_codec.h"
#include "config.h"
#include "esp32_camera.h"
#include "manual_control_server.h"
#include "mcp_server.h"
#include "wall_e_portrait_display.h"

namespace {
constexpr char kTag[] = "WallEAI";
constexpr uint8_t kPcaAddress = 0x40;

extern const uint8_t wall_e_voice_start[] asm("_binary_wallevoice_wav_start");
extern const uint8_t wall_e_voice_end[] asm("_binary_wallevoice_wav_end");

uint16_t ReadLe16(const uint8_t* data) {
    return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t ReadLe32(const uint8_t* data) {
    return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
}

class Pca9685 {
public:
    explicit Pca9685(i2c_master_bus_handle_t bus) : bus_(bus) {
        i2c_device_config_t config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = kPcaAddress,
            .scl_speed_hz = 400000,
        };
        ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_, &config, &device_));
        Write(0x00, 0x10);
        vTaskDelay(pdMS_TO_TICKS(5));
        Write(0xfe, 121);  // 50 Hz
        Write(0x00, 0x20);
        vTaskDelay(pdMS_TO_TICKS(5));
        Write(0x00, 0xa1);
        for (uint8_t channel = 0; channel < 16; ++channel)
            SetDuty(channel, 0);
    }

    void SetDuty(uint8_t channel, uint16_t duty) {
        if (channel > 15)
            return;
        duty = std::min<uint16_t>(duty, 4095);
        const uint8_t reg = static_cast<uint8_t>(0x06 + channel * 4);
        uint8_t data[] = {reg, 0, 0, 0, 0};
        if (duty == 0) {
            data[4] = 0x10;
        } else {
            data[3] = static_cast<uint8_t>(duty);
            data[4] = static_cast<uint8_t>(duty >> 8);
        }
        const esp_err_t err = i2c_master_transmit(device_, data, sizeof(data), 100);
        if (err != ESP_OK) {
            ESP_LOGW(kTag, "PCA9685 channel %u write failed: %s", channel, esp_err_to_name(err));
        }
    }

    void SetServo(uint8_t channel, int angle) {
        angle = std::clamp(angle, 0, 180);
        const int pulse_us = 500 + (2000 * angle) / 180;
        SetDuty(channel, static_cast<uint16_t>((pulse_us * 4096 * 50) / 1000000));
    }

private:
    void Write(uint8_t reg, uint8_t value) {
        const uint8_t data[] = {reg, value};
        const esp_err_t err = i2c_master_transmit(device_, data, sizeof(data), 100);
        if (err != ESP_OK) {
            ESP_LOGW(kTag, "PCA9685 register 0x%02x write failed: %s", reg, esp_err_to_name(err));
        }
    }

    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t device_ = nullptr;
};

class WallEAudioCodec : public NoAudioCodecSimplex {
public:
    WallEAudioCodec()
        : NoAudioCodecSimplex(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                              AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK,
                              AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK,
                              AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN) {}

protected:
    int Read(int16_t* dest, int samples) override {
        const int count = NoAudioCodec::Read(dest, samples);
        // The INMP441 is physically a little far from the mouth opening on this
        // build.  A modest 1.5x digital pre-gain helps WakeNet hear "Hi Wall-E"
        // without the heavy clipping that a larger gain would introduce.
        for (int i = 0; i < count; ++i) {
            const int amplified = (static_cast<int>(dest[i]) * 3) / 2;
            dest[i] = static_cast<int16_t>(std::clamp(amplified, -32768, 32767));
        }
        return count;
    }
};

}  // namespace

class WallEAiBoard : public WifiBoard {
public:
    WallEAiBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        InitializeDisplay();
        pca_ = new Pca9685(i2c_bus_);
        InitializeCamera();
        manual_control_ = new ManualControlServer(
            [this](unsigned char channel, unsigned short duty) { pca_->SetDuty(channel, duty); },
            camera_->GetMutex(), [this]() { ExitManualControl(); });
        manual_control_->Initialize();
        InitializeButton();
        InitializeMcpTools();
        ESP_LOGI(kTag, "Wall-E AI ready: Xiaozhi is the default mode");
    }

    AudioCodec* GetAudioCodec() override {
        static WallEAudioCodec codec;
        return &codec;
    }

    Display* GetDisplay() override { return display_; }
    Camera* GetCamera() override { return camera_; }
    bool IsManualControlActive() const override {
        return manual_control_ != nullptr && manual_control_->active();
    }

    void OnAssistantSpeaking(bool speaking) override {
        if (manual_control_ != nullptr)
            manual_control_->SetAiSpeaking(speaking);
    }

    void OnAssistantEmotion(const std::string& emotion) override {
        if (manual_control_ != nullptr)
            manual_control_->SetAiEmotion(emotion);
    }

    void OnUserText(const std::string& text) override {
        std::string normalized = text;
        for (char& character : normalized) {
            if (character >= 'A' && character <= 'Z')
                character = static_cast<char>(character - 'A' + 'a');
        }
        const bool manual_intent = normalized.find("lái thủ công") != std::string::npos ||
                                   normalized.find("lái bằng tay") != std::string::npos ||
                                   normalized.find("điều khiển thủ công") != std::string::npos ||
                                   normalized.find("chế độ lái") != std::string::npos ||
                                   normalized.find("manual control") != std::string::npos ||
                                   normalized.find("manual driving") != std::string::npos;
        if (!manual_intent || manual_start_pending_.exchange(true))
            return;
        Application::GetInstance().Schedule([this]() {
            StartManualControl();
            manual_start_pending_.store(false);
        });
    }

    StartupAudio GetStartupAudio() override {
        const size_t file_size = static_cast<size_t>(wall_e_voice_end - wall_e_voice_start);
        if (file_size < 12 || std::memcmp(wall_e_voice_start, "RIFF", 4) != 0 ||
            std::memcmp(wall_e_voice_start + 8, "WAVE", 4) != 0) {
            ESP_LOGW(kTag, "Startup sound is not a RIFF/WAVE file");
            return {};
        }

        uint16_t format = 0;
        uint16_t channels = 0;
        uint16_t bits_per_sample = 0;
        uint32_t sample_rate = 0;
        const uint8_t* pcm_data = nullptr;
        size_t pcm_size = 0;
        for (size_t offset = 12; offset + 8 <= file_size;) {
            const uint8_t* chunk = wall_e_voice_start + offset;
            const size_t chunk_size = ReadLe32(chunk + 4);
            const size_t payload_offset = offset + 8;
            if (chunk_size > file_size - payload_offset)
                break;
            if (std::memcmp(chunk, "fmt ", 4) == 0 && chunk_size >= 16) {
                format = ReadLe16(wall_e_voice_start + payload_offset);
                channels = ReadLe16(wall_e_voice_start + payload_offset + 2);
                sample_rate = ReadLe32(wall_e_voice_start + payload_offset + 4);
                bits_per_sample = ReadLe16(wall_e_voice_start + payload_offset + 14);
            } else if (std::memcmp(chunk, "data", 4) == 0) {
                pcm_data = wall_e_voice_start + payload_offset;
                pcm_size = chunk_size;
            }
            offset = payload_offset + chunk_size + (chunk_size & 1U);
        }
        if (format != 1 || channels != 1 || bits_per_sample != 16 || sample_rate == 0 ||
            pcm_data == nullptr || pcm_size < sizeof(int16_t)) {
            ESP_LOGW(kTag, "Startup sound must be mono 16-bit PCM");
            return {};
        }
        return {reinterpret_cast<const int16_t*>(pcm_data), pcm_size / sizeof(int16_t),
                static_cast<int>(sample_rate)};
    }

    void OnStartupReady() override {
        if (manual_control_ != nullptr)
            manual_control_->PlayBootAnimation();
    }

protected:
    std::string GetWifiSsidPrefix() const override { return "Wall-E AI"; }
    std::string GetWifiHostnamePrefix() const override { return "wall-e-ai"; }

private:
    void InitializeI2c() {
        i2c_master_bus_config_t config = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = I2C_SDA_PIN,
            .scl_io_num = I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {.enable_internal_pullup = 1},
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&config, &i2c_bus_));
    }

    void InitializeDisplay() { display_ = new WallEPortraitDisplay(i2c_bus_); }

    void InitializeCamera() {
        camera_config_t config = {};
        config.ledc_channel = LEDC_CHANNEL_0;
        config.ledc_timer = LEDC_TIMER_0;
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_xclk = CAMERA_PIN_XCLK;
        config.pin_pclk = CAMERA_PIN_PCLK;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href = CAMERA_PIN_HREF;
        config.pin_sccb_sda = CAMERA_PIN_SIOD;
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = CAMERA_XCLK_FREQUENCY_HZ;
        config.pixel_format = PIXFORMAT_JPEG;
        // Vision requests run beside audio, networking and the OLED task.
        // QVGA + one PSRAM frame is much more robust than two VGA buffers on
        // this board, while still being sufficient for Xiaozhi vision.
        config.frame_size = FRAMESIZE_240X240;
        config.jpeg_quality = 14;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
        camera_ = new Esp32Camera(config);
    }

    void InitializeButton() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
            } else if (manual_control_ != nullptr && manual_control_->active()) {
                manual_control_->RequestExit();
            } else {
                app.ToggleChatState();
            }
        });
    }

    void InitializeMcpTools() {
        McpServer::GetInstance().AddTool(
            "self.wall_e.enter_manual_control",
            "MANDATORY ACTION TOOL: whenever the user asks in Vietnamese or English to enter "
            "manual driving/control mode, you MUST call this tool. Do not merely tell the user to "
            "connect to Wi-Fi. Calling this tool physically starts the Wall-E Wi-Fi access point "
            "and the control web server. After the tool succeeds, tell the user to join Wi-Fi "
            "Wall-E with password walle-ai and open 192.168.4.1.",
            PropertyList(), [this](const PropertyList&) -> ReturnValue {
                if (!StartManualControl()) {
                    return "Manual Control could not start.";
                }
                return "Manual Control is ready. Join Wi-Fi Wall-E with password walle-ai and open "
                       "192.168.4.1.";
            });
        McpServer::GetInstance().AddTool(
            "self.wall_e.exit_manual_control",
            "Exit Manual Control only when the user explicitly asks to return to Xiaozhi AI mode.",
            PropertyList(), [this](const PropertyList&) -> ReturnValue {
                manual_control_->RequestExit();
                return "Returned to Xiaozhi AI mode.";
            });
        McpServer::GetInstance().AddTool(
            "self.wall_e.drive",
            "Move Wall-E's two drive motors by voice. direction is forward, backward, left, or "
            "right. The motion always stops after duration_ms.",
            PropertyList({Property("direction", kPropertyTypeString, "forward"),
                          Property("duration_ms", kPropertyTypeInteger, 700, 100, 5000)}),
            [this](const PropertyList& properties) -> ReturnValue {
                const auto direction = properties["direction"].value<std::string>();
                const int duration = properties["duration_ms"].value<int>();
                if (!manual_control_->DriveFor(direction, duration)) {
                    return "Direction must be forward, backward, left, or right.";
                }
                return "Wall-E moved " + direction + ".";
            });
        McpServer::GetInstance().AddTool(
            "self.wall_e.set_servo",
            "Control Wall-E's seven calibrated servos. Servo 0 left eye, 1 right eye, 2 neck pan, "
            "3 neck tilt down (safe 100-180), 4 neck tilt up (safe 112-180), 5 left arm, 6 right "
            "arm.",
            PropertyList({Property("servo", kPropertyTypeInteger, 0, 0, 6),
                          Property("angle", kPropertyTypeInteger, 90, 0, 180)}),
            [this](const PropertyList& properties) -> ReturnValue {
                return manual_control_->SetServoAngle(properties["servo"].value<int>(),
                                                      properties["angle"].value<int>())
                           ? "Servo moved."
                           : "Invalid servo.";
            });
        McpServer::GetInstance().AddTool(
            "self.wall_e.pose",
            "Set Wall-E pose. pose must be normal, sad, angry, arms_up, arms_down, or neck_home.",
            PropertyList({Property("pose", kPropertyTypeString, "normal")}),
            [this](const PropertyList& properties) -> ReturnValue {
                return manual_control_->ApplyNamedPose(properties["pose"].value<std::string>())
                           ? "Pose applied."
                           : "Invalid pose.";
            });
    }

    bool StartManualControl() {
        if (manual_control_ == nullptr)
            return false;
        auto& app = Application::GetInstance();
        app.GetAudioService().EnableVoiceProcessing(false);
        app.GetAudioService().EnableWakeWordDetection(false);
        if (!manual_control_->Start()) {
            if (app.GetDeviceState() == kDeviceStateIdle)
                app.GetAudioService().EnableWakeWordDetection(true);
            return false;
        }
        const auto state = app.GetDeviceState();
        if (state == kDeviceStateSpeaking)
            app.AbortSpeaking(kAbortReasonNone);
        if (state == kDeviceStateSpeaking || state == kDeviceStateListening ||
            state == kDeviceStateConnecting) {
            app.SetDeviceState(kDeviceStateIdle);
        }
        display_->ShowNotification("MANUAL: WIFI WALL-E");
        ESP_LOGI(kTag, "Manual mode entered locally; Wi-Fi Wall-E is available");
        return true;
    }

    void ExitManualControl() {
        Application::GetInstance().Schedule([this]() {
            manual_control_->Stop();
            auto& app = Application::GetInstance();
            app.GetAudioService().EnableVoiceProcessing(false);
            app.GetAudioService().EnableWakeWordDetection(true);
            display_->ShowNotification("XIAOZHI AI");
        });
    }

    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    Button boot_button_;
    Display* display_ = nullptr;
    Esp32Camera* camera_ = nullptr;
    Pca9685* pca_ = nullptr;
    ManualControlServer* manual_control_ = nullptr;
    std::atomic_bool manual_start_pending_{false};
};

DECLARE_BOARD(WallEAiBoard);
