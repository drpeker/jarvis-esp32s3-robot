#include "power.h"

#include <algorithm>
#include <mutex>

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "i2c_bus.h"
#include "motors.h"
#include "pins.h"

#define TAG "power"

namespace {
constexpr int kShuntMilliohm = 100;
constexpr int kLowMv = 3500, kCriticalMv = 3300, kRecoverMv = 3600;

i2c_master_dev_handle_t g_dev = nullptr;
std::mutex g_mutex;
power::Reading g_reading;
int g_rest_mv = 0;

bool ReadReg(uint8_t reg, int16_t& out) {
    uint8_t d[2];
    if (i2c_master_transmit_receive(g_dev, &reg, 1, d, 2, 50) != ESP_OK) return false;
    out = (int16_t)(d[0] << 8 | d[1]);
    return true;
}

// Resting cell voltage -> percent (typical Li-ion curve).
int Percent(int mv) {
    static const int kMv[] = {3300, 3500, 3600, 3700, 3750, 3800, 3900, 4000, 4100, 4150};
    static const int kPct[] = {0, 5, 12, 25, 40, 52, 68, 82, 95, 100};
    if (mv <= kMv[0]) return 0;
    for (int i = 1; i < 10; ++i) {
        if (mv <= kMv[i]) return kPct[i - 1] + (kPct[i] - kPct[i - 1]) * (mv - kMv[i - 1]) / (kMv[i] - kMv[i - 1]);
    }
    return 100;
}

void Task(void*) {
    int64_t motors_stopped_since = 0;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        int16_t bus = 0, shunt = 0;
        const bool ok = ReadReg(0x02, bus) && ReadReg(0x01, shunt);
        const int mv = ((uint16_t)bus >> 3) * 4;
        const int ma = shunt * 10 / kShuntMilliohm;  // shunt LSB 10 uV
        const int64_t now = esp_timer_get_time();
        if (motors::Moving()) motors_stopped_since = now;
        const bool resting = now - motors_stopped_since > 2000000;
        if (ok && resting) g_rest_mv = g_rest_mv == 0 ? mv : (g_rest_mv * 3 + mv) / 4;

        std::lock_guard<std::mutex> lock(g_mutex);
        power::Reading& r = g_reading;
        r.ok = ok;
        if (!ok) continue;
        r.mv = mv;
        r.ma = ma;
        if (g_rest_mv > 0) {
            r.percent = Percent(g_rest_mv);
            r.level = r.percent < 10 ? 3 : r.percent < 20 ? 2 : r.percent < 35 ? 1 : 0;
            r.low = g_rest_mv < kLowMv;
            if (g_rest_mv < kCriticalMv && !r.critical) {
                r.critical = true;
                motors::SetLocked(true);
                ESP_LOGW(TAG, "battery critical (%d mV): motors locked", g_rest_mv);
            } else if (r.critical && g_rest_mv > kRecoverMv) {
                r.critical = false;
                motors::SetLocked(false);
                ESP_LOGI(TAG, "battery recovered (%d mV): motors unlocked", g_rest_mv);
            }
        }
    }
}
}  // namespace

namespace power {

void Start() {
    g_dev = i2c_bus::AddDevice(INA219_ADDR);
    if (g_dev == nullptr) {
        ESP_LOGW(TAG, "INA219 not found");
        return;
    }
    // 32 V range, PGA /8, 12-bit, continuous shunt + bus.
    const uint8_t cfg[3] = {0x00, 0x39, 0x9F};
    i2c_master_transmit(g_dev, cfg, 3, 50);
    xTaskCreate(Task, "power", 2560, nullptr, 2, nullptr);
}

Reading Get() {
    std::lock_guard<std::mutex> lock(g_mutex);
    return g_reading;
}

}  // namespace power
