#include "body_sense.h"

#include <cmath>
#include <cstdio>

#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/idf_additions.h>
#include <freertos/task.h>

#include "application.h"
#include "display/display.h"
#include "motor_controller.h"
#include "mpu6050.h"

#define TAG "BodySense"

namespace {
constexpr int kPeriodTicks = 5;           // 50 ms at 100 Hz tick
constexpr float kTipDeg = 35.0f;          // tipped over above this tilt ...
constexpr int kTipSamples = 10;           // ... for 0.5 s
constexpr float kUprightDeg = 15.0f;      // upright again below this ...
constexpr int kUprightSamples = 20;       // ... for 1 s
// Lifted / held by hand (motors off): |a| away from the rest value, the body turning or
// tilting away from how it stood. A hand is never as still as the floor.
constexpr float kLiftAccelG = 0.12f;
constexpr float kLiftGyroDps = 25.0f;
constexpr float kLiftTiltDeg = 10.0f;
constexpr int kLiftSamples = 2;
constexpr float kRestAccelG = 0.04f;      // set down: still and level ...
constexpr float kRestGyroDps = 3.0f;
constexpr float kRestTiltDeg = 6.0f;
constexpr int kRestSamples = 30;          // ... for 1.5 s
constexpr float kBumpG = 1.2f;            // sideways/forward jolt while driving ...
constexpr int64_t kStartIgnoreUs = 500000;  // ... but not the motors' own start jolt
constexpr int64_t kMotorQuietUs = 600000;
constexpr int64_t kBumpDebounceUs = 2000000;
constexpr int64_t kAiDebounceUs = 8000000;

const char* EventName(int event) {
    static const char* const kNames[] = {"none", "lifted", "put_down", "tipped_over", "upright",
                                         "bump"};
    return kNames[event];
}

float Length(const float v[3]) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

float Dot(const float a[3], const float b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
}  // namespace

BodySense::BodySense(Mpu6050* imu, MotorController* motor, Display* display)
    : imu_(imu), motor_(motor), display_(display) {
    xTaskCreateWithCaps([](void* arg) { static_cast<BodySense*>(arg)->Loop(); }, "body_sense",
                        3072, this, 2, nullptr, MALLOC_CAP_SPIRAM);
}

void BodySense::Loop() {
    while (!imu_->HeadingOk()) {
        vTaskDelay(pdMS_TO_TICKS(100));  // "up" and rest g come from the IMU calibration
    }
    const float* up = imu_->Up();
    float lp[3] = {up[0], up[1], up[2]};
    int tip_count = 0, upright_count = 0, lift_count = 0, rest_count = 0;
    int64_t last_motor_us = 0, last_bump_us = 0, motor_start_us = 0;
    bool was_moving = false;
    const float rest_g = imu_->RestG();
    ESP_LOGI(TAG, "Body sense ready: rest %.3f g", rest_g);

    while (true) {
        vTaskDelay(kPeriodTicks);
        float accel[3], gyro[3];
        if (!imu_->ReadMotion(accel, gyro)) {
            continue;
        }
        const int64_t now = esp_timer_get_time();
        const bool moving = motor_->IsMoving();
        if (moving) {
            if (!was_moving) {
                motor_start_us = now;
            }
            last_motor_us = now;
        }
        was_moving = moving;
        const bool motors_recent = now - last_motor_us < kMotorQuietUs;

        // Jolt = change against the slow average, without the vertical part.
        float jolt[3];
        for (int k = 0; k < 3; ++k) {
            jolt[k] = accel[k] - lp[k];
        }
        const float vertical = Dot(jolt, up);
        for (int k = 0; k < 3; ++k) {
            jolt[k] -= vertical * up[k];
        }
        for (int k = 0; k < 3; ++k) {
            lp[k] += (accel[k] - lp[k]) * 0.3f;
        }

        const float lp_len = Length(lp);
        const float tilt =
            lp_len > 0.2f
                ? std::acos(std::fmax(-1.0f, std::fmin(1.0f, Dot(lp, up) / lp_len))) * 180.0f /
                      static_cast<float>(M_PI)
                : 90.0f;
        tilt_x10_.store(static_cast<int>(tilt * 10));
        const float accel_off = std::fabs(Length(accel) - rest_g);
        const float spin = Length(gyro);

        // Tipping over / back upright.
        tip_count = tilt > kTipDeg ? tip_count + 1 : 0;
        upright_count = tilt < kUprightDeg ? upright_count + 1 : 0;
        if (!tipped_ && tip_count >= kTipSamples) {
            tipped_ = true;
            char detail[32];
            snprintf(detail, sizeof(detail), "%.0f deg", tilt);
            Report(kTipped, detail);
        } else if (tipped_ && upright_count >= kUprightSamples) {
            tipped_ = false;
            Report(kUpright, "");
        }

        // Lifted / moved by hand (only with the motors quiet), and set down again.
        if (!held_ && !motors_recent) {
            lift_count = (accel_off > kLiftAccelG || spin > kLiftGyroDps || tilt > kLiftTiltDeg)
                             ? lift_count + 1
                             : 0;
            if (lift_count >= kLiftSamples) {
                held_ = true;
                rest_count = 0;
                char detail[48];
                snprintf(detail, sizeof(detail), "%.2f g, %.0f deg/s, %.0f deg", accel_off, spin,
                         tilt);
                Report(kLifted, detail);
            }
        } else if (held_) {
            rest_count = (accel_off < kRestAccelG && spin < kRestGyroDps && tilt < kRestTiltDeg)
                             ? rest_count + 1
                             : 0;
            if (rest_count >= kRestSamples) {
                held_ = false;
                lift_count = 0;
                Report(kPutDown, "");
            }
        }

        // Bump while driving.
        const float jolt_g = Length(jolt);
        if (moving && now - motor_start_us > kStartIgnoreUs && jolt_g > kBumpG &&
            now - last_bump_us > kBumpDebounceUs) {
            last_bump_us = now;
            char detail[24];
            snprintf(detail, sizeof(detail), "%.2f g", jolt_g);
            Report(kBump, detail);
        }
    }
}

void BodySense::Report(Event event, const char* detail) {
    const int64_t now = esp_timer_get_time();
    last_event_.store(event);
    last_event_us_.store(now);
    ESP_LOGW(TAG, "Body event: %s %s", EventName(event), detail);

    const bool alarming = event == kLifted || event == kTipped || event == kBump;
    if (alarming) {
        motor_->Stop();
        if (display_ != nullptr) {
            display_->SetEmotion("surprised");
        }
    }
    if (event == kUpright) {
        return;
    }

    // Hand the event to the AI (it decides what to say/do). Only possible while idle: the
    // message starts a new conversation turn.
    auto& app = Application::GetInstance();
    if (app.GetDeviceState() != kDeviceStateIdle || now - last_ai_event_us_ < kAiDebounceUs) {
        return;
    }
    last_ai_event_us_ = now;
    const char* text = nullptr;
    switch (event) {
        case kLifted: text = "Olay: Jarvis, biri seni yerden kaldirdi ya da elle oynatti."; break;
        case kPutDown: text = "Olay: Jarvis, yere geri kondun."; break;
        case kTipped: text = "Olay: Jarvis, dengen bozuldu, yan yattin."; break;
        case kBump: text = "Olay: Jarvis, ilerlerken bir yere carptin, motorlar durdu."; break;
        default: break;
    }
    if (text != nullptr) {
        app.WakeWordInvoke(text);
    }
}

std::string BodySense::StatusJson() const {
    const int64_t since = last_event_.load() == kNone
                              ? -1
                              : (esp_timer_get_time() - last_event_us_.load()) / 1000000;
    char buffer[200];
    snprintf(buffer, sizeof(buffer),
             "{\"tilt_deg\":%.1f,\"lifted_or_held\":%s,\"tipped_over\":%s,\"last_event\":\"%s\","
             "\"last_event_seconds_ago\":%lld,\"imu_mount_tilt_deg\":%.1f}",
             tilt_x10_.load() / 10.0f, held_ ? "true" : "false", tipped_ ? "true" : "false",
             EventName(last_event_.load()), static_cast<long long>(since),
             imu_->MountTiltDegrees());
    return buffer;
}
