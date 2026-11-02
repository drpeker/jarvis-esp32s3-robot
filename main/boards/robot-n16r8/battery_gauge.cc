#include "battery_gauge.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/idf_additions.h>
#include <freertos/task.h>

#include "application.h"
#include "config.h"
#include "settings.h"

#define TAG "BatteryGauge"

namespace {
// Resting cell voltage -> percent, same table as NeuroVision-Gemini (per cell).
constexpr float kCurve[][2] = {
    {3.27f, 0},  {3.61f, 5},  {3.69f, 10}, {3.71f, 15}, {3.73f, 20}, {3.75f, 25}, {3.77f, 30},
    {3.79f, 35}, {3.80f, 40}, {3.82f, 45}, {3.84f, 50}, {3.85f, 55}, {3.87f, 60}, {3.91f, 65},
    {3.95f, 70}, {3.98f, 75}, {4.02f, 80}, {4.08f, 85}, {4.11f, 90}, {4.15f, 95}, {4.20f, 100},
};
constexpr int kLevelBands[] = {35, 20, 10};
constexpr int kLevelBack = 3;  // go back up a level only this many % above the band
constexpr int kFailsMax = 5;
constexpr float kMaxStepS = 5.0f;
constexpr int64_t kSaveUs = 120LL * 1000 * 1000;
constexpr float kAvgS = 60.0f;
constexpr float kChargedV = 0.25f / 3;  // Gemini used 0.25 V for 3S; per cell
constexpr char kNamespace[] = "battery";
}  // namespace

BatteryGauge::BatteryGauge(Ina219* sensor) : sensor_(sensor) {
    // NVS is touched only from tasks with internal stacks: read here (board init), written
    // through the main task. The polling task itself runs on a PSRAM stack.
    Settings settings(kNamespace, false);
    saved_charge_x100_ = settings.GetInt("charge_x100", -1);
    saved_mv_ = settings.GetInt("volts_mv", -1);
    xTaskCreateWithCaps([](void* arg) { static_cast<BatteryGauge*>(arg)->Loop(); },
                        "battery_gauge", 3072, this, 2, nullptr, MALLOC_CAP_SPIRAM);
}

BatteryGauge::Snapshot BatteryGauge::Get() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
}

void BatteryGauge::Loop() {
    while (true) {
        int bus_mv = 0, current_ma = 0, power_mw = 0;
        if (sensor_->Read(bus_mv, current_ma, power_mw)) {
            fails_ = 0;
            const float volts = bus_mv / 1000.0f;
            const float amps = current_ma / 1000.0f;
            std::lock_guard<std::mutex> lock(mutex_);
            snapshot_.volts = volts;
            snapshot_.amps = amps;
            snapshot_.watts = power_mw / 1000.0f;
            snapshot_.peak_amps = std::max(snapshot_.peak_amps, std::fabs(amps));
            Count(volts, amps, esp_timer_get_time());
            snapshot_.ok = true;
        } else if (++fails_ == kFailsMax) {
            std::lock_guard<std::mutex> lock(mutex_);
            snapshot_.ok = false;
            ESP_LOGW(TAG, "INA219 stopped answering");
        }
        vTaskDelay(pdMS_TO_TICKS(BATTERY_POLL_MS));
    }
}

void BatteryGauge::Count(float volts, float amps, int64_t now_us) {
    const bool idle = std::fabs(amps) * 1000.0f < BATTERY_IDLE_MA;
    const float volts_percent = PercentOfVolts(volts);
    if (charge_ < 0.0f) {
        charge_ = LoadCharge(volts, volts_percent, idle);
        snapshot_.avg_amps = amps;
    } else {
        const float dt = (now_us - read_at_us_) / 1e6f;
        if (dt <= kMaxStepS) {
            charge_ -= amps * dt / 3.6f / BATTERY_CAPACITY_MAH * 100.0f;
            snapshot_.avg_amps += (amps - snapshot_.avg_amps) * dt / kAvgS;
            if (idle) {
                charge_ += (volts_percent - charge_) * dt / BATTERY_BLEND_S;
            }
        }
    }
    if (idle && volts / BATTERY_CELLS * 1000.0f >= BATTERY_FULL_CELL_MV) {
        charge_ = 100.0f;
    }
    charge_ = std::clamp(charge_, 0.0f, 100.0f);
    read_at_us_ = now_us;
    snapshot_.percent = static_cast<int>(std::lround(charge_));
    UpdateLevel(snapshot_.percent);
    if (idle && now_us - saved_at_us_ > kSaveUs) {
        saved_at_us_ = now_us;
        SaveCharge(volts);
    }
}

float BatteryGauge::PercentOfVolts(float volts) const {
    const float cell = volts / BATTERY_CELLS;
    constexpr size_t n = sizeof(kCurve) / sizeof(kCurve[0]);
    if (cell <= kCurve[0][0]) {
        return 0.0f;
    }
    for (size_t i = 1; i < n; ++i) {
        if (cell <= kCurve[i][0]) {
            const float v0 = kCurve[i - 1][0], p0 = kCurve[i - 1][1];
            const float v1 = kCurve[i][0], p1 = kCurve[i][1];
            return std::round(p0 + (p1 - p0) * (cell - v0) / (v1 - v0));
        }
    }
    return 100.0f;
}

void BatteryGauge::UpdateLevel(int percent) {
    int level = 0;
    for (int band : kLevelBands) {
        level += percent < band ? 1 : 0;
    }
    const int current = snapshot_.level;
    if (level > current ||
        (level < current && percent >= kLevelBands[current - 1] + kLevelBack)) {
        snapshot_.level = level;
    }
}

float BatteryGauge::LoadCharge(float volts, float volts_percent, bool idle) {
    const int saved_charge = saved_charge_x100_;
    const int saved_mv = saved_mv_;
    if (idle && saved_charge >= 0 && saved_mv > 0 &&
        std::fabs(volts - saved_mv / 1000.0f) / BATTERY_CELLS < kChargedV) {
        ESP_LOGI(TAG, "Battery charge resumed from NVS: %.1f%%", saved_charge / 100.0f);
        return saved_charge / 100.0f;
    }
    ESP_LOGI(TAG, "Battery charge from voltage (%.2f V): %d%%", volts,
             static_cast<int>(volts_percent));
    return volts_percent;
}

void BatteryGauge::SaveCharge(float volts) {
    const int charge_x100 = static_cast<int>(charge_ * 100.0f);
    const int volts_mv = static_cast<int>(volts * 1000.0f);
    Application::GetInstance().Schedule([charge_x100, volts_mv]() {
        Settings settings(kNamespace, true);
        settings.SetInt("charge_x100", charge_x100);
        settings.SetInt("volts_mv", volts_mv);
    });
}

std::string BatteryGauge::StatusJson() const {
    static const char* const kLevels[] = {"good", "getting_low", "low", "very_low"};
    const Snapshot s = Get();
    if (!s.ok) {
        return "{\"state\":\"no_measurement (INA219 not answering)\"}";
    }
    const float left_mah = s.percent / 100.0f * BATTERY_CAPACITY_MAH;
    char buffer[384];
    snprintf(buffer, sizeof(buffer),
             "{\"percent\":%d,\"level\":\"%s\",\"capacity_mah\":%d,\"remaining_mah\":%d,"
             "\"used_mah\":%d,\"average_current_a\":%.3f,\"voltage_v\":%.2f,\"cell_v\":%.2f,"
             "\"current_a\":%.3f,\"power_w\":%.2f,\"peak_current_a\":%.2f,"
             "\"note\":\"Measured. Percent is counted from the current drawn; current is "
             "instant, average is over the last minute. Work out the remaining time "
             "yourself.\"}",
             s.percent, kLevels[s.level], BATTERY_CAPACITY_MAH, static_cast<int>(left_mah),
             static_cast<int>(BATTERY_CAPACITY_MAH - left_mah), s.avg_amps, s.volts,
             s.volts / BATTERY_CELLS, s.amps, s.watts, s.peak_amps);
    return buffer;
}
