#include "motors.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <mutex>

#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "imu.h"
#include "pins.h"
#include "settings.h"

#define TAG "motors"

namespace {
constexpr ledc_mode_t kMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kTimer = LEDC_TIMER_2;
constexpr uint32_t kMaxDuty = (1 << 10) - 1;
constexpr ledc_channel_t kCh[4] = {LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3, LEDC_CHANNEL_4};
constexpr gpio_num_t kPins[4] = {PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2};
constexpr int kDeadTimeMs = 120, kStaggerMs = 40, kSettleMs = 300, kMaxRunMs = 5000;
constexpr float kToleranceDeg = 3.0f;
constexpr const char* kNs = "robot";
const char* kMinKeys[2][2] = {{"min_lf", "min_lr"}, {"min_rf", "min_rr"}};

motors::Tuning g_t;
std::mutex g_move;               // one move at a time
std::atomic<bool> g_stop{false};
std::atomic<bool> g_moving{false};
std::atomic<bool> g_locked{false};
float g_lead = 1.5f;             // learned coast, degrees
int g_last[2] = {0, 0};          // last physical speed per motor

void SetDuty(int ch, uint32_t duty) {
    ledc_set_duty(kMode, kCh[ch], duty);
    ledc_update_duty(kMode, kCh[ch]);
}

// speed -100..100 on motor m (0 left, 1 right). raw: no dead-band mapping (kick pulses).
void SetMotor(int m, int speed, bool raw = false) {
    int pct = std::min(std::abs(speed), 100);
    if (!raw && pct > 0) {
        const int min = g_t.min_duty[m][speed > 0 ? 0 : 1];
        const int scaled = pct * g_t.gain[m] / 100;
        pct = min + (100 - min) * std::max(scaled, 1) / 100;
    }
    const uint32_t duty = pct * kMaxDuty / 100;
    SetDuty(m * 2, speed > 0 ? duty : 0);
    SetDuty(m * 2 + 1, speed < 0 ? duty : 0);
}

void SetNow(int left, int right) {
    SetMotor(0, left);
    SetMotor(1, right);
    g_last[0] = left;
    g_last[1] = right;
    g_moving.store(left != 0 || right != 0);
}

// Start from rest the gentle way: dead time before a polarity change, a full-power kick to
// break static friction, and the two motors 40 ms apart (inrush).
void Apply(int left, int right) {
    const int want[2] = {left, right};
    bool reversing = false;
    for (int m = 0; m < 2; ++m) {
        reversing |= g_last[m] != 0 && want[m] != 0 && (g_last[m] > 0) != (want[m] > 0);
    }
    if (reversing) {
        SetNow(0, 0);
        vTaskDelay(pdMS_TO_TICKS(kDeadTimeMs));
    }
    bool kick = false;
    for (int m = 0; m < 2; ++m) kick |= g_t.kick_ms > 0 && g_last[m] == 0 && want[m] != 0 && std::abs(want[m]) < 100;
    if (kick) {
        auto full = [](int s) { return s > 0 ? 100 : s < 0 ? -100 : 0; };
        SetMotor(0, full(left), true);
        if (left && right) vTaskDelay(pdMS_TO_TICKS(kStaggerMs));
        SetMotor(1, full(right), true);
        vTaskDelay(pdMS_TO_TICKS(g_t.kick_ms));
    }
    SetMotor(0, left);
    if (!kick && g_last[0] == 0 && g_last[1] == 0 && left && right) vTaskDelay(pdMS_TO_TICKS(kStaggerMs));
    SetMotor(1, right);
    g_last[0] = left;
    g_last[1] = right;
    g_moving.store(left != 0 || right != 0);
}

// MX1508 brake: both inputs high short the motor.
void BrakeOn() {
    for (int ch = 0; ch < 4; ++ch) SetDuty(ch, kMaxDuty);
}

// Waits ms (in 5 ms steps); false if Stop() was called.
bool Wait(int ms, float* heading_out = nullptr) {
    const int64_t end = esp_timer_get_time() + ms * 1000LL;
    while (esp_timer_get_time() < end) {
        if (g_stop.load()) return false;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return true;
}

void Load() {
    for (int m = 0; m < 2; ++m)
        for (int d = 0; d < 2; ++d) g_t.min_duty[m][d] = settings::GetInt(kNs, kMinKeys[m][d], 80);
    g_t.gain[0] = settings::GetInt(kNs, "gain_l", 100);
    g_t.gain[1] = settings::GetInt(kNs, "gain_r", 100);
    g_t.turn_speed = settings::GetInt(kNs, "turn_spd", 70);
    g_t.lead_x10 = settings::GetInt(kNs, "lead_x10", 15);
    g_t.kick_ms = settings::GetInt(kNs, "kick_ms", 60);
    g_t.brake_ms = settings::GetInt(kNs, "brake_ms", 80);
    g_t.pwm_hz = settings::GetInt(kNs, "pwm_hz", 200);
    g_t.dist_speed = settings::GetInt(kNs, "dist_spd", 50);
    g_t.ms20[0] = settings::GetInt(kNs, "ms20_fwd", 1050);
    g_t.ms20[1] = settings::GetInt(kNs, "ms20_back", 1000);
    g_lead = g_t.lead_x10 / 10.0f;
}

const char* Begin() {
    if (g_locked.load()) return "motors locked: battery critical, charge it";
    g_stop.store(false);
    return nullptr;
}
}  // namespace

namespace motors {

void Start() {
    Load();
    ledc_timer_config_t tc = {};
    tc.speed_mode = kMode;
    tc.duty_resolution = LEDC_TIMER_10_BIT;
    tc.timer_num = kTimer;
    tc.freq_hz = g_t.pwm_hz;
    tc.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&tc));
    for (int i = 0; i < 4; ++i) {
        ledc_channel_config_t cc = {};
        cc.gpio_num = kPins[i];
        cc.speed_mode = kMode;
        cc.channel = kCh[i];
        cc.timer_sel = kTimer;
        cc.duty = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&cc));
    }
    ESP_LOGI(TAG, "min duty L %d/%d R %d/%d, turn %d%%, lead %.1f, kick %d, brake %d, PWM %d Hz, 20 cm %d/%d ms",
             g_t.min_duty[0][0], g_t.min_duty[0][1], g_t.min_duty[1][0], g_t.min_duty[1][1],
             g_t.turn_speed, g_lead, g_t.kick_ms, g_t.brake_ms, g_t.pwm_hz, g_t.ms20[0], g_t.ms20[1]);
}

// Closed-loop turn (see header). Without the IMU: 6 ms per degree.
Result Turn(bool left, int degrees) {
    std::lock_guard<std::mutex> lock(g_move);
    if (const char* why = Begin()) return {false, 0, why};
    degrees = std::clamp(degrees, 1, 720);
    const int s = g_t.turn_speed;
    const float side = left ? 1.0f : -1.0f;
    auto spin = [&](float toward) {
        const int v = (int)(side * toward) * s;
        SetNow(-v, v);  // left turn: left track back, right forward
    };
    if (!imu::Ready()) {
        spin(1);
        const bool done = Wait(std::min(degrees * 6, kMaxRunMs));
        SetNow(0, 0);
        return {done, done ? (float)degrees : 0, done ? "timed (no IMU)" : "stopped"};
    }
    const float start = imu::Heading();
    auto progress = [&]() { return (imu::Heading() - start) * side; };
    auto brake_settle = [&]() {
        BrakeOn();
        const bool ok = Wait(g_t.brake_ms);
        SetNow(0, 0);
        return ok && Wait(kSettleMs);
    };
    const float target = degrees, lead = g_lead;
    const int64_t deadline = esp_timer_get_time() + std::min<int64_t>(kMaxRunMs, 600 + degrees * 1000 / 45) * 1000;
    spin(1);
    bool reached = false;
    while (!g_stop.load()) {
        if (progress() >= target - lead) {
            reached = true;
            break;
        }
        if (esp_timer_get_time() > deadline) break;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    const float at_stop = progress();
    if (g_stop.load() || !brake_settle()) {
        SetNow(0, 0);
        return {false, progress(), "stopped"};
    }
    if (reached) g_lead = std::clamp(0.6f * g_lead + 0.4f * (progress() - at_stop), 0.0f, 25.0f);
    int corrections = 0;
    while (reached && corrections < 4) {
        const float err = target - progress();
        if (std::fabs(err) <= kToleranceDeg) break;
        ++corrections;
        spin(err > 0 ? 1 : -1);
        if (!Wait(std::clamp((int)(std::fabs(err) * 5), 40, 160)) || !brake_settle()) {
            SetNow(0, 0);
            return {false, progress(), "stopped"};
        }
    }
    const float final_deg = progress();
    ESP_LOGI(TAG, "turn %s %d: lead %.1f at_stop %.1f corrections %d final %.1f (next lead %.1f)",
             left ? "left" : "right", degrees, lead, at_stop, corrections, final_deg, g_lead);
    return {reached, final_deg, reached ? "" : "timed out: robot may be stuck"};
}

Result Distance(bool forward, int cm) {
    const int ms = std::clamp(cm, 1, 200) * g_t.ms20[forward ? 0 : 1] / 20;
    const int run_ms = std::min(ms, kMaxRunMs);
    Result r = Drive(forward, run_ms, g_t.dist_speed);
    // Time based: the planned distance when the run finished, else the share that ran
    // (Drive's own time also counts the start kick and the brake, so it is not used).
    r.value = r.ok ? std::clamp(cm, 1, 200) * run_ms / std::max(ms, 1) : r.value * 20 / g_t.ms20[forward ? 0 : 1];
    return r;
}

Result Drive(bool forward, int ms, int speed) {
    std::lock_guard<std::mutex> lock(g_move);
    if (const char* why = Begin()) return {false, 0, why};
    ms = std::clamp(ms, 50, kMaxRunMs);
    speed = std::clamp(speed, 1, 100) * (forward ? 1 : -1);
    const int64_t t0 = esp_timer_get_time();
    Apply(speed, speed);
    const bool done = Wait(ms);
    BrakeOn();  // distance moves stop the same way every time
    vTaskDelay(pdMS_TO_TICKS(g_t.brake_ms));
    SetNow(0, 0);
    return {done, (float)((esp_timer_get_time() - t0) / 1000), done ? "" : "stopped"};
}

void Stop() {
    g_stop.store(true);
    SetNow(0, 0);
}

bool Moving() { return g_moving.load(); }
void SetLocked(bool locked) {
    g_locked.store(locked);
    if (locked) Stop();
}
bool Locked() { return g_locked.load(); }
Tuning GetTuning() { return g_t; }
float TurnLead() { return g_lead; }

void SetTuning(const Tuning& t, bool save) {
    const int old_hz = g_t.pwm_hz;
    g_t = t;
    for (auto& m : g_t.min_duty)
        for (int& d : m) d = std::clamp(d, 80, 95);  // user rule: never below 80 %
    for (int& g : g_t.gain) g = std::clamp(g, 30, 100);
    g_t.turn_speed = std::clamp(g_t.turn_speed, 10, 100);
    g_t.lead_x10 = std::clamp(g_t.lead_x10, 0, 300);
    g_t.kick_ms = std::clamp(g_t.kick_ms, 0, 300);
    g_t.brake_ms = std::clamp(g_t.brake_ms, 0, 300);
    g_t.pwm_hz = std::clamp(g_t.pwm_hz, 100, 20000);
    g_t.dist_speed = std::clamp(g_t.dist_speed, 1, 100);
    for (int& ms : g_t.ms20) ms = std::clamp(ms, 100, 5000);
    if (g_t.pwm_hz != old_hz) ledc_set_freq(kMode, kTimer, g_t.pwm_hz);
    if (!save) return;
    for (int m = 0; m < 2; ++m)
        for (int d = 0; d < 2; ++d) settings::SetInt(kNs, kMinKeys[m][d], g_t.min_duty[m][d]);
    settings::SetInt(kNs, "gain_l", g_t.gain[0]);
    settings::SetInt(kNs, "gain_r", g_t.gain[1]);
    settings::SetInt(kNs, "turn_spd", g_t.turn_speed);
    settings::SetInt(kNs, "lead_x10", g_t.lead_x10);
    settings::SetInt(kNs, "kick_ms", g_t.kick_ms);
    settings::SetInt(kNs, "brake_ms", g_t.brake_ms);
    settings::SetInt(kNs, "pwm_hz", g_t.pwm_hz);
    settings::SetInt(kNs, "dist_spd", g_t.dist_speed);
    settings::SetInt(kNs, "ms20_fwd", g_t.ms20[0]);
    settings::SetInt(kNs, "ms20_back", g_t.ms20[1]);
}

}  // namespace motors
