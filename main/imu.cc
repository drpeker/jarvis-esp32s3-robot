#include "imu.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "i2c_bus.h"
#include "pins.h"
#include "settings.h"

#define TAG "imu"

namespace {
constexpr uint8_t kRateDiv = 0x19, kConfig = 0x1A, kGyroConfig = 0x1B, kAccelConfig = 0x1C,
                  kIntEnable = 0x38, kData = 0x3B, kSignalReset = 0x68, kPwr1 = 0x6B;
constexpr float kLsbPerDps = 65.5f;   // +-500 dps
constexpr float kLsbPerG = 16384.0f;  // +-2 g
constexpr int kPeriodMs = 10;
constexpr int kCalSamples = 150;      // 1.5 s

i2c_master_dev_handle_t g_dev = nullptr;
float g_bias[3] = {}, g_up[3] = {0, 0, 1}, g_rest_g = 1, g_lp[3] = {0, 0, 1};
std::atomic<float> g_heading{0}, g_rate{0}, g_tilt{0}, g_scale{1};
std::atomic<bool> g_ready{false};
std::atomic<int> g_errors{0};
std::mutex g_debug_mutex;
int g_still = 0;

bool Write(uint8_t reg, uint8_t v) {
    const uint8_t d[2] = {reg, v};
    return i2c_master_transmit(g_dev, d, 2, 50) == ESP_OK;
}

float Norm(const float v[3]) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

bool Configure() {
    if (!Write(kPwr1, 0x80)) return false;  // full reset: a warm restart keeps old registers
    vTaskDelay(pdMS_TO_TICKS(100));
    Write(kSignalReset, 0x07);
    vTaskDelay(pdMS_TO_TICKS(100));
    const bool ok = Write(kPwr1, 0x01) &&       // wake, PLL on gyro X
                    Write(kConfig, 0x03) &&     // DLPF 44 Hz (motor vibration)
                    Write(kRateDiv, 9) &&       // 100 Hz internal sample rate
                    Write(kGyroConfig, 0x08) && Write(kAccelConfig, 0x00) && Write(kIntEnable, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    return ok;
}

// One aligned sample (accel g, gyro dps). false on a bus error or an implausible value.
bool Sample(float a[3], float g[3]) {
    uint8_t d[14];
    const uint8_t reg = kData;
    if (i2c_master_transmit_receive(g_dev, &reg, 1, d, sizeof(d), 50) != ESP_OK) {
        g_errors.fetch_add(1);
        return false;
    }
    for (int k = 0; k < 3; ++k) {
        a[k] = (int16_t)(d[k * 2] << 8 | d[k * 2 + 1]) / kLsbPerG;
        g[k] = (int16_t)(d[8 + k * 2] << 8 | d[9 + k * 2]) / kLsbPerDps;
    }
    const float n = Norm(a);
    if (n < 0.3f || n > 3.0f) {  // bus noise
        g_errors.fetch_add(1);
        return false;
    }
    return true;
}

// 1.5 s average while the robot stands still; false if it moved (try again).
bool Calibrate(bool last_try) {
    double sa[3] = {}, sg[3] = {}, qg[3] = {};
    int n = 0;
    TickType_t wake = xTaskGetTickCount();
    while (n < kCalSamples) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(kPeriodMs));
        float a[3], g[3];
        if (!Sample(a, g)) continue;
        for (int k = 0; k < 3; ++k) {
            sa[k] += a[k];
            sg[k] += g[k];
            qg[k] += g[k] * g[k];
        }
        ++n;
    }
    float std_g = 0;
    for (int k = 0; k < 3; ++k) {
        const double m = sg[k] / n;
        std_g = std::max(std_g, (float)std::sqrt(std::max(0.0, qg[k] / n - m * m)));
    }
    if (std_g > 0.6f && !last_try) {
        ESP_LOGW(TAG, "calibration: robot moved (gyro std %.2f), again", std_g);
        return false;
    }
    const float mean[3] = {(float)(sa[0] / n), (float)(sa[1] / n), (float)(sa[2] / n)};
    const float m = Norm(mean);
    for (int k = 0; k < 3; ++k) {
        g_bias[k] = sg[k] / n;
        g_up[k] = m > 0 ? mean[k] / m : (k == 2);
        g_lp[k] = mean[k];
    }
    g_rest_g = m > 0 ? m : 1;
    return true;
}

void Process(const float a[3], const float graw[3], float dt) {
    float g[3];
    for (int k = 0; k < 3; ++k) g[k] = graw[k] - g_bias[k];
    const float rate = (g[0] * g_up[0] + g[1] * g_up[1] + g[2] * g_up[2]) * g_scale.load();
    g_heading.store(g_heading.load() + rate * dt);
    g_rate.store(rate);
    // Refine the bias after 2 s of stillness (temperature drift).
    const bool still = std::fabs(g[0]) < 1 && std::fabs(g[1]) < 1 && std::fabs(g[2]) < 1 &&
                       std::fabs(Norm(a) - g_rest_g) < 0.02f;
    g_still = still ? g_still + 1 : 0;
    if (g_still > 200) {
        for (int k = 0; k < 3; ++k) g_bias[k] += 0.002f * g[k];
    }
    std::lock_guard<std::mutex> lock(g_debug_mutex);
    for (int k = 0; k < 3; ++k) g_lp[k] += (a[k] - g_lp[k]) * 0.05f;
    const float ln = Norm(g_lp);
    if (ln > 0.2f) {
        const float c = (g_lp[0] * g_up[0] + g_lp[1] * g_up[1] + g_lp[2] * g_up[2]) / ln;
        g_tilt.store(std::acos(std::clamp(c, -1.0f, 1.0f)) * 57.2958f);
    }
}

void Task(void*) {
    for (int attempt = 1; !Calibrate(attempt >= 5); ++attempt) {
    }
    g_ready.store(true);
    ESP_LOGI(TAG, "calibrated: bias %.2f/%.2f/%.2f dps, up (%.2f %.2f %.2f) |g| %.3f, yaw scale %.4f", g_bias[0],
             g_bias[1], g_bias[2], g_up[0], g_up[1], g_up[2], g_rest_g, g_scale.load());
    TickType_t wake = xTaskGetTickCount();
    int64_t last = esp_timer_get_time();
    while (true) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(kPeriodMs));
        float a[3], g[3];
        const int64_t now = esp_timer_get_time();
        if (!Sample(a, g)) continue;  // the rate of the last good sample bridges the gap
        const float dt = std::min((now - last) / 1e6f, 0.05f);
        last = now;
        Process(a, g, dt);
    }
}
}  // namespace

namespace imu {

bool Start() {
    g_dev = i2c_bus::AddDevice(MPU6050_ADDR);
    if (g_dev == nullptr || !Configure()) {
        ESP_LOGW(TAG, "MPU-6050 not available; turns use timing");
        return false;
    }
    g_scale.store(settings::GetInt("robot", "gyro_scale", 10000) / 10000.0f);
    xTaskCreatePinnedToCore(Task, "imu", 3072, nullptr, 6, nullptr, 1);
    return true;
}

bool Ready() { return g_ready.load(); }
float Heading() { return g_heading.load(); }
float YawRate() { return g_rate.load(); }
float TiltDegrees() { return g_tilt.load(); }
int ReadErrors() { return g_errors.load(); }
float YawScale() { return g_scale.load(); }

void SetYawScale(float scale) {
    scale = std::clamp(scale, 0.5f, 1.5f);
    g_scale.store(scale);
    settings::SetInt("robot", "gyro_scale", (int)std::lround(scale * 10000));
}

void Debug(float up[3], float accel[3]) {
    std::lock_guard<std::mutex> lock(g_debug_mutex);
    for (int k = 0; k < 3; ++k) {
        up[k] = g_up[k];
        accel[k] = g_lp[k];
    }
}

}  // namespace imu
