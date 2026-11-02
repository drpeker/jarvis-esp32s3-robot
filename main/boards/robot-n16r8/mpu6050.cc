#include "mpu6050.h"

#include <algorithm>
#include <cmath>

#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/idf_additions.h>
#include <freertos/task.h>

#include "settings.h"

#define TAG "Mpu6050"

namespace {
// Registers (MPU-6050 register map rev 4.2).
constexpr uint8_t kRegSampleRateDiv = 0x19;
constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegFifoEnable = 0x23;
constexpr uint8_t kRegIntEnable = 0x38;
constexpr uint8_t kRegSignalPathReset = 0x68;
constexpr uint8_t kRegUserCtrl = 0x6A;
constexpr uint8_t kRegPowerMgmt1 = 0x6B;
constexpr uint8_t kRegFifoCountHigh = 0x72;
constexpr uint8_t kRegFifoData = 0x74;
constexpr uint8_t kRegWhoAmI = 0x75;

constexpr uint8_t kUserCtrlFifoEnable = 0x40;
constexpr uint8_t kUserCtrlFifoReset = 0x04;
constexpr uint8_t kFifoAccelAndGyro = 0x78;  // XG | YG | ZG | ACCEL

// DLPF 44 Hz -> gyro output 1 kHz; divider 9 -> 100 Hz into the FIFO.
constexpr uint8_t kDlpf44Hz = 0x03;
constexpr uint8_t kRateDiv100Hz = 9;
constexpr float kDt = 0.01f;
constexpr uint8_t kGyro500Dps = 0x08;
constexpr float kLsbPerDps = 65.5f;
constexpr uint8_t kAccel2G = 0x00;
constexpr float kLsbPerG = 16384.0f;

constexpr int kFrameBytes = 12;
constexpr int kFifoSize = 1024;
constexpr int kFramesPerRead = 5;  // 60 bytes, ~6 ms on the bus at 100 kHz
constexpr int kTimeoutMs = 50;
constexpr int kDrainTicks = 3;     // 30 ms

// Calibration: 1.5 s of samples, must be still.
constexpr int kCalibrationFrames = 150;
constexpr float kStillGyroStd = 0.6f;    // deg/s
constexpr float kStillAccelStd = 0.015f; // g
constexpr int kCalibrationAttempts = 5;

// Bias refinement: after 2 s of stillness, follow the gyro slowly.
constexpr float kRefineGyroDps = 1.0f;
constexpr float kRefineAccelG = 0.02f;
constexpr int kRefineAfterSamples = 200;
constexpr float kRefineRate = 0.002f;

constexpr int kOdometerBiasSamples = 20;

// Frame check: gravity is always in the accel part of a frame, so |a| must be within
// 0.5..2 g. Misaligned frames mix high/low bytes of different words and fail this.
constexpr float kFrameMinG = 0.6f;
constexpr float kFrameMaxG = 1.6f;

float Norm(const float v[3]) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }
}  // namespace

Mpu6050::Mpu6050(i2c_master_bus_handle_t bus, uint8_t address) {
    if (i2c_master_probe(bus, address, 50) != ESP_OK) {
        ESP_LOGW(TAG, "IMU not found at 0x%02X; turns fall back to timing", address);
        return;
    }
    i2c_device_config_t config = {};
    config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    config.device_address = address;
    config.scl_speed_hz = 100000;
    if (i2c_master_bus_add_device(bus, &config, &device_) != ESP_OK) {
        device_ = nullptr;
        ESP_LOGE(TAG, "Failed to add IMU device");
        return;
    }
    uint8_t who_am_i = 0;
    ReadRegister(kRegWhoAmI, &who_am_i, 1);
    if (!Configure()) {
        ESP_LOGE(TAG, "IMU configuration failed; turns fall back to timing");
        i2c_master_bus_rm_device(device_);
        device_ = nullptr;
        return;
    }
    Settings settings("robot", false);
    SetYawScale(settings.GetInt("gyro_scale", 10000) / 10000.0f);
    ESP_LOGI(TAG, "IMU at 0x%02X (WHO_AM_I=0x%02X), 100 Hz FIFO, calibrating...", address,
             who_am_i);
    xTaskCreateWithCaps([](void* arg) { static_cast<Mpu6050*>(arg)->Run(); }, "imu", 4096, this,
                        5, nullptr, MALLOC_CAP_SPIRAM);
}

// Full reset to a known state (a warm restart keeps the old register values).
bool Mpu6050::Configure() {
    if (!WriteRegister(kRegPowerMgmt1, 0x80)) {  // DEVICE_RESET
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    WriteRegister(kRegSignalPathReset, 0x07);
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!WriteRegister(kRegPowerMgmt1, 0x01) ||  // wake, PLL on gyro X
        !WriteRegister(kRegConfig, kDlpf44Hz) ||
        !WriteRegister(kRegSampleRateDiv, kRateDiv100Hz) ||
        !WriteRegister(kRegGyroConfig, kGyro500Dps) ||
        !WriteRegister(kRegAccelConfig, kAccel2G) || !WriteRegister(kRegIntEnable, 0x00)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    ResetFifo();
    fifo_resets_.store(0);
    return true;
}

// Datasheet order: stop the FIFO, reset it, then enable it again. Resetting while it is
// running can leave a partial frame and shift every later frame.
void Mpu6050::ResetFifo() {
    WriteRegister(kRegUserCtrl, 0x00);
    WriteRegister(kRegFifoEnable, 0x00);
    WriteRegister(kRegUserCtrl, kUserCtrlFifoReset);
    WriteRegister(kRegFifoEnable, kFifoAccelAndGyro);
    WriteRegister(kRegUserCtrl, kUserCtrlFifoEnable);
    fifo_resets_.fetch_add(1);
}

// Reads whole frames only. Returns the number of frames decoded.
int Mpu6050::ReadFrames(float accel[][3], float gyro[][3], int max_frames) {
    uint8_t count_bytes[2] = {};
    if (!ReadRegister(kRegFifoCountHigh, count_bytes, 2)) {
        return 0;
    }
    const int count = count_bytes[0] << 8 | count_bytes[1];
    if (count > kFifoSize - kFrameBytes) {
        // Full: the chip may already have overwritten data, frames are not trustworthy.
        ESP_LOGW(TAG, "IMU FIFO full (%d bytes), reset", count);
        ResetFifo();
        return 0;
    }
    const int frames = std::min(count / kFrameBytes, max_frames);
    int decoded = 0;
    uint8_t data[kFrameBytes * kFramesPerRead];
    while (decoded < frames) {
        const int n = std::min(frames - decoded, kFramesPerRead);
        if (!ReadRegister(kRegFifoData, data, n * kFrameBytes)) {
            // Unknown how many bytes left the FIFO: realign from scratch.
            ESP_LOGW(TAG, "IMU FIFO read failed, reset");
            ResetFifo();
            break;
        }
        for (int f = 0; f < n; ++f) {
            const uint8_t* frame = data + f * kFrameBytes;
            for (int k = 0; k < 3; ++k) {
                accel[decoded + f][k] =
                    static_cast<int16_t>(frame[k * 2] << 8 | frame[k * 2 + 1]) / kLsbPerG;
                gyro[decoded + f][k] =
                    static_cast<int16_t>(frame[6 + k * 2] << 8 | frame[7 + k * 2]) / kLsbPerDps;
            }
        }
        decoded += n;
    }
    return decoded;
}

void Mpu6050::Run() {
    for (int attempt = 1; !Calibrate(attempt); ++attempt) {
    }
    calibrated_.store(true);
    ESP_LOGI(TAG,
             "IMU calibrated: gyro bias %.2f/%.2f/%.2f dps, up (%.2f, %.2f, %.2f), rest %.3f g, "
             "tilt %.1f deg, yaw scale %.4f",
             bias_[0], bias_[1], bias_[2], up_[0], up_[1], up_[2], rest_g_, MountTiltDegrees(),
             yaw_scale_);

    float accel[16][3], gyro[16][3];
    int64_t last_report_us = esp_timer_get_time();
    int good_frames = 0;
    while (true) {
        vTaskDelay(kDrainTicks);
        const int n = ReadFrames(accel, gyro, 16);
        for (int i = 0; i < n; ++i) {
            const float g = Norm(accel[i]);
            if (g < kFrameMinG || g > kFrameMaxG) {
                // First implausible frame: the stream slipped (bus error). Drop the rest of
                // the batch and realign.
                ESP_LOGW(TAG, "IMU frames misaligned (|a| %.2f g), FIFO reset #%d after %d good",
                         g, fifo_resets_.load(), good_frames);
                ResetFifo();
                good_frames = 0;
                break;
            }
            Process(accel[i], gyro[i]);
            ++good_frames;
        }
        const int64_t now_us = esp_timer_get_time();
        if (now_us - last_report_us > 60000000LL) {
            last_report_us = now_us;
            ESP_LOGI(TAG, "IMU: %lu samples, %d FIFO resets", (unsigned long)sample_count_.load(),
                     fifo_resets_.load());
        }
    }
}

// Averages 1.5 s of samples. Returns false (try again) if the robot was not still.
bool Mpu6050::Calibrate(int attempt) {
    ResetFifo();
    double sum_a[3] = {}, sum_g[3] = {}, sq_g[3] = {}, sum_n = 0, sq_n = 0;
    int total = 0;
    float accel[16][3], gyro[16][3];
    while (total < kCalibrationFrames) {
        vTaskDelay(kDrainTicks);
        const int n = ReadFrames(accel, gyro, 16);
        for (int i = 0; i < n && total < kCalibrationFrames; ++i, ++total) {
            const float norm = Norm(accel[i]);
            sum_n += norm;
            sq_n += norm * norm;
            for (int k = 0; k < 3; ++k) {
                sum_a[k] += accel[i][k];
                sum_g[k] += gyro[i][k];
                sq_g[k] += gyro[i][k] * gyro[i][k];
            }
        }
    }
    float gyro_std = 0;
    for (int k = 0; k < 3; ++k) {
        const double mean = sum_g[k] / total;
        gyro_std = std::max(gyro_std, static_cast<float>(std::sqrt(
                                          std::max(0.0, sq_g[k] / total - mean * mean))));
    }
    const double mean_n = sum_n / total;
    const float accel_std =
        static_cast<float>(std::sqrt(std::max(0.0, sq_n / total - mean_n * mean_n)));
    const bool still = gyro_std < kStillGyroStd && accel_std < kStillAccelStd;
    if (!still && attempt < kCalibrationAttempts) {
        ESP_LOGW(TAG, "IMU calibration %d: robot moved (gyro std %.2f, accel std %.3f), again",
                 attempt, gyro_std, accel_std);
        return false;
    }
    if (!still) {
        ESP_LOGW(TAG, "IMU calibrated while moving; drift possible until it rests");
    }
    const float mean_a[3] = {static_cast<float>(sum_a[0] / total),
                             static_cast<float>(sum_a[1] / total),
                             static_cast<float>(sum_a[2] / total)};
    const float g = Norm(mean_a);
    for (int k = 0; k < 3; ++k) {
        bias_[k] = static_cast<float>(sum_g[k] / total);
        up_[k] = g > 0 ? mean_a[k] / g : (k == 2 ? 1.0f : 0.0f);
    }
    rest_g_ = g > 0 ? g : 1.0f;
    return true;
}

void Mpu6050::Process(const float accel[3], const float gyro_raw[3]) {
    float gyro[3];
    for (int k = 0; k < 3; ++k) {
        gyro[k] = gyro_raw[k] - bias_[k];
    }
    const float rate = (gyro[0] * up_[0] + gyro[1] * up_[1] + gyro[2] * up_[2]) * yaw_scale_;
    heading_.store(heading_.load() + rate * kDt);

    // Refine the gyro bias while the robot stays still (temperature drift).
    const bool still = std::fabs(gyro[0]) < kRefineGyroDps && std::fabs(gyro[1]) < kRefineGyroDps &&
                       std::fabs(gyro[2]) < kRefineGyroDps &&
                       std::fabs(Norm(accel) - rest_g_) < kRefineAccelG;
    still_samples_ = still ? still_samples_ + 1 : 0;
    if (still_samples_ > kRefineAfterSamples) {
        for (int k = 0; k < 3; ++k) {
            bias_[k] += kRefineRate * gyro[k];
        }
    }

    {
        std::lock_guard<std::mutex> lock(latest_mutex_);
        for (int k = 0; k < 3; ++k) {
            latest_accel_[k] = accel[k];
            latest_gyro_[k] = gyro[k];
        }
        latest_yaw_rate_ = rate;
    }
    sample_count_.fetch_add(1);

    // Odometer: same integration as before, now on every 10 ms sample.
    const int state = odo_state_.load();
    if (state == kOdoOff) {
        return;
    }
    const float along = accel[odo_axis_] * odo_sign_;
    if (state == kOdoBias) {
        odo_bias_sum_ += along;
        if (++odo_bias_count_ >= kOdometerBiasSamples) {
            odo_bias_ = odo_bias_sum_ / odo_bias_count_;
            odo_state_.store(kOdoRunning);
        }
        return;
    }
    const float acceleration = (along - odo_bias_) * 9.81f * odo_travel_;
    const float speed = std::max(0.0f, odo_speed_.load() + acceleration * kDt);
    odo_speed_.store(speed);
    odo_distance_.store(odo_distance_.load() + speed * kDt);
}

void Mpu6050::StartOdometer(int axis, int sign, float travel) {
    odo_state_.store(kOdoOff);
    odo_axis_ = axis == 1 ? 1 : 0;
    odo_sign_ = sign < 0 ? -1.0f : 1.0f;
    odo_travel_ = travel < 0 ? -1.0f : 1.0f;
    odo_bias_sum_ = 0;
    odo_bias_count_ = 0;
    odo_speed_.store(0);
    odo_distance_.store(0);
    odo_state_.store(kOdoBias);
}

bool Mpu6050::ReadYawRate(float& dps) {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    dps = latest_yaw_rate_;
    return calibrated_.load();
}

bool Mpu6050::ReadMotion(float accel_g[3], float gyro_dps[3]) {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    for (int k = 0; k < 3; ++k) {
        accel_g[k] = latest_accel_[k];
        gyro_dps[k] = latest_gyro_[k];
    }
    return calibrated_.load();
}

bool Mpu6050::ReadAccelXY(float& x_g, float& y_g) {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    x_g = latest_accel_[0];
    y_g = latest_accel_[1];
    return calibrated_.load();
}

float Mpu6050::MountTiltDegrees() const {
    return std::acos(std::fmin(1.0f, std::fabs(up_[2]))) * 180.0f / static_cast<float>(M_PI);
}

void Mpu6050::SetYawScale(float scale) { yaw_scale_ = std::fmin(1.5f, std::fmax(0.5f, scale)); }

void Mpu6050::SaveYawScale() const {
    Settings settings("robot", true);
    settings.SetInt("gyro_scale", static_cast<int>(std::lround(yaw_scale_ * 10000)));
}

bool Mpu6050::ReadRegister(uint8_t reg, uint8_t* buffer, size_t length) {
    return i2c_master_transmit_receive(device_, &reg, 1, buffer, length, kTimeoutMs) == ESP_OK;
}

bool Mpu6050::WriteRegister(uint8_t reg, uint8_t value) {
    const uint8_t buffer[2] = {reg, value};
    return i2c_master_transmit(device_, buffer, 2, kTimeoutMs) == ESP_OK;
}
