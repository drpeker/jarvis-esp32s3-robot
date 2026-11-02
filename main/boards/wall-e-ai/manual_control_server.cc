#include "manual_control_server.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>
#include <mutex>
#include <utility>

extern "C" {
#include <esp_camera.h>
#include <esp_err.h>
#include <esp_http_server.h>
#include <esp_netif.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <cJSON.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs.h>
}

#include <esp_log.h>

namespace {
constexpr char kTag[] = "ManualControl";
constexpr uint8_t kServoCount = 7;
constexpr uint8_t kPoseCount = 6;
constexpr uint8_t kLeftRpwm = 8;
constexpr uint8_t kLeftLpwm = 9;
constexpr uint8_t kRightRpwm = 10;
constexpr uint8_t kRightLpwm = 11;
constexpr char kManualSsid[] = "Wall-E";
constexpr char kManualPassword[] = "walle-ai";

enum ServoId : uint8_t { kEyeLeft, kEyeRight, kPan, kTilt1, kTilt2, kArmLeft, kArmRight };
enum PoseId : uint8_t { kNormal, kSad, kAngry, kArmsUp, kArmsDown, kNeckHome };

struct ServoConfig {
    uint8_t channel;
    uint8_t reverse;
    uint16_t pulse_min_us;
    uint16_t pulse_max_us;
    uint8_t home_angle;
};

struct PersistedSettings {
    uint32_t magic;
    uint16_t version;
    uint8_t motor_reverse_left;
    uint8_t motor_reverse_right;
    ServoConfig servo[kServoCount];
    int8_t pose_offsets[kPoseCount][kServoCount];
};

constexpr uint32_t kMagic = 0x57414c4c;
constexpr uint16_t kVersion = 4;
constexpr const char* kServoNames[] = {"eye_left",    "eye_right", "neck_pan", "neck_tilt_1",
                                       "neck_tilt_2", "arm_left",  "arm_right"};
constexpr const char* kPoseNames[] = {"normal",  "sad",       "angry",
                                      "arms_up", "arms_down", "neck_home"};

ManualControlServer* g_server = nullptr;
ManualControlServer::PwmWriter g_write_pwm;
PersistedSettings g_settings{};
uint8_t g_angles[kServoCount]{};
httpd_handle_t g_http = nullptr;
httpd_handle_t g_stream = nullptr;
bool g_ap_created = false;
std::mutex* g_camera_mutex = nullptr;
std::recursive_mutex g_state_mutex;
std::recursive_mutex g_motion_mutex;
std::atomic<int64_t> g_last_drive_us{0};
std::atomic<int64_t> g_motion_deadline_us{0};
std::atomic_bool g_motors_running{false};
TaskHandle_t g_motor_safety_task = nullptr;
TaskHandle_t g_ai_animation_task = nullptr;
std::atomic_bool g_ai_speaking{false};
std::atomic<int> g_ai_emotion{0};
std::atomic_bool g_ai_motor_animation{false};
wifi_ps_type_t g_previous_wifi_ps = WIFI_PS_MIN_MODEM;
bool g_previous_wifi_ps_valid = false;

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");

int ClampAngle(uint8_t id, int angle) {
    if (id == kTilt1)
        return std::clamp(angle, 100, 180);
    if (id == kTilt2)
        return std::clamp(angle, 112, 180);
    return std::clamp(angle, 0, 180);
}

bool IsMotorChannel(int channel) {
    return channel == kLeftRpwm || channel == kLeftLpwm || channel == kRightRpwm ||
           channel == kRightLpwm;
}

bool ValidateSettings(PersistedSettings& settings) {
    bool used_channels[16]{};
    settings.motor_reverse_left = settings.motor_reverse_left ? 1 : 0;
    settings.motor_reverse_right = settings.motor_reverse_right ? 1 : 0;
    for (uint8_t id = 0; id < kServoCount; ++id) {
        ServoConfig& servo = settings.servo[id];
        if (servo.channel > 15 || IsMotorChannel(servo.channel) || used_channels[servo.channel]) {
            return false;
        }
        used_channels[servo.channel] = true;
        servo.reverse = servo.reverse ? 1 : 0;
        if (servo.pulse_min_us < 400 || servo.pulse_min_us > 2000 || servo.pulse_max_us < 1000 ||
            servo.pulse_max_us > 2800 || servo.pulse_min_us + 100 >= servo.pulse_max_us) {
            return false;
        }
        servo.home_angle = static_cast<uint8_t>(ClampAngle(id, servo.home_angle));
        for (uint8_t pose = 0; pose < kPoseCount; ++pose) {
            settings.pose_offsets[pose][id] =
                std::clamp<int>(settings.pose_offsets[pose][id], -90, 90);
        }
    }
    return true;
}

void Defaults() {
    std::memset(&g_settings, 0, sizeof(g_settings));
    g_settings.magic = kMagic;
    g_settings.version = kVersion;
    // The two gear motors are installed as mirrored left/right units.  Their
    // electrical polarity therefore has to be opposite for physical forward.
    // This value is persisted in the same wall_e NVS blob used by Manual and AI.
    g_settings.motor_reverse_right = 1;
    for (uint8_t id = 0; id < kServoCount; ++id) {
        g_settings.servo[id] = {id, 0, 500, 2500, static_cast<uint8_t>(ClampAngle(id, 90))};
        g_angles[id] = g_settings.servo[id].home_angle;
    }
    g_settings.pose_offsets[kSad][kEyeLeft] = 18;
    g_settings.pose_offsets[kSad][kEyeRight] = -18;
    g_settings.pose_offsets[kAngry][kEyeLeft] = -25;
    g_settings.pose_offsets[kAngry][kEyeRight] = 25;
    g_settings.pose_offsets[kArmsUp][kArmLeft] = -45;
    g_settings.pose_offsets[kArmsUp][kArmRight] = 45;
}

void Load() {
    std::lock_guard<std::recursive_mutex> lock(g_state_mutex);
    Defaults();
    nvs_handle_t handle;
    if (nvs_open("wall_e", NVS_READONLY, &handle) != ESP_OK)
        return;
    PersistedSettings restored{};
    size_t size = sizeof(restored);
    const esp_err_t err = nvs_get_blob(handle, "settings", &restored, &size);
    nvs_close(handle);
    if (err == ESP_OK && size == sizeof(restored) && restored.magic == kMagic &&
        restored.version == kVersion && ValidateSettings(restored)) {
        g_settings = restored;
        for (uint8_t id = 0; id < kServoCount; ++id)
            g_angles[id] = g_settings.servo[id].home_angle;
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(kTag, "Ignoring invalid Wall-E settings: %s", esp_err_to_name(err));
    }
}

esp_err_t Save() {
    std::lock_guard<std::recursive_mutex> lock(g_state_mutex);
    nvs_handle_t handle;
    esp_err_t err = nvs_open("wall_e", NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return err;
    err = nvs_set_blob(handle, "settings", &g_settings, sizeof(g_settings));
    if (err == ESP_OK)
        err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK)
        ESP_LOGE(kTag, "Could not save settings: %s", esp_err_to_name(err));
    return err;
}

void SetDuty(uint8_t channel, uint16_t duty) {
    if (g_write_pwm)
        g_write_pwm(channel, duty);
}

void SetServo(uint8_t id, int angle) {
    std::lock_guard<std::recursive_mutex> state_lock(g_state_mutex);
    if (id >= kServoCount)
        return;
    angle = ClampAngle(id, angle);
    const ServoConfig& s = g_settings.servo[id];
    const int electrical_angle = s.reverse ? 180 - angle : angle;
    const int pulse = s.pulse_min_us + ((s.pulse_max_us - s.pulse_min_us) * electrical_angle) / 180;
    const uint16_t duty = static_cast<uint16_t>(std::clamp((pulse * 4096 * 50) / 1000000, 1, 4095));
    g_angles[id] = angle;
    SetDuty(s.channel, duty);
}

void StopMotors() {
    std::lock_guard<std::recursive_mutex> motion_lock(g_motion_mutex);
    SetDuty(kLeftRpwm, 0);
    SetDuty(kLeftLpwm, 0);
    SetDuty(kRightRpwm, 0);
    SetDuty(kRightLpwm, 0);
    g_motors_running.store(false);
    g_motion_deadline_us.store(0);
}

void Motor(bool right, int power) {
    std::lock_guard<std::recursive_mutex> motion_lock(g_motion_mutex);
    std::lock_guard<std::recursive_mutex> state_lock(g_state_mutex);
    power = std::clamp(power, -100, 100);
    if ((right && g_settings.motor_reverse_right) || (!right && g_settings.motor_reverse_left))
        power = -power;
    const uint8_t r = right ? kRightRpwm : kLeftRpwm;
    const uint8_t l = right ? kRightLpwm : kLeftLpwm;
    const uint16_t duty = static_cast<uint16_t>((std::abs(power) * 4095) / 100);
    if (power > 2) {
        SetDuty(r, duty);
        SetDuty(l, 0);
    } else if (power < -2) {
        SetDuty(r, 0);
        SetDuty(l, duty);
    } else {
        SetDuty(r, 0);
        SetDuty(l, 0);
    }
}

void Drive(int x, int y) {
    std::lock_guard<std::recursive_mutex> motion_lock(g_motion_mutex);
    // The installed chassis is mirrored relative to the original coordinate
    // system, so both the longitudinal and steering axes need inversion.
    // Both Xiaozhi and Manual Control call this shared function, so their
    // directions always remain consistent.
    Motor(false, std::clamp(-y - x, -100, 100));
    Motor(true, std::clamp(-y + x, -100, 100));
    g_last_drive_us.store(esp_timer_get_time());
    g_motion_deadline_us.store(0);
    g_motors_running.store(std::abs(x) > 2 || std::abs(y) > 2);
}

enum AiEmotionId : int { kAiNeutral, kAiHappy, kAiSad, kAiAngry, kAiCurious, kAiSurprised };

bool AiAnimationAllowed() {
    return g_ai_speaking.load() && g_server != nullptr && !g_server->active();
}

std::array<int, kServoCount> HomeTargets() {
    std::array<int, kServoCount> target{};
    std::lock_guard<std::recursive_mutex> state_lock(g_state_mutex);
    for (uint8_t id = 0; id < kServoCount; ++id)
        target[id] = g_settings.servo[id].home_angle;
    return target;
}

std::array<int, kServoCount> ReactionTargets(int emotion, int phase) {
    auto target = HomeTargets();
    std::lock_guard<std::recursive_mutex> state_lock(g_state_mutex);
    const int direction = phase == 0 ? 1 : -1;
    const auto add_pose = [&](uint8_t pose) {
        for (uint8_t id = 0; id < kServoCount; ++id)
            target[id] += g_settings.pose_offsets[pose][id];
    };
    switch (emotion) {
        case kAiHappy:
            target[kEyeLeft] -= 7 * direction;
            target[kEyeRight] += 7 * direction;
            target[kPan] += 9 * direction;
            target[kArmLeft] -= phase == 0 ? 16 : 7;
            target[kArmRight] += phase == 0 ? 16 : 7;
            break;
        case kAiSad:
            add_pose(kSad);
            target[kPan] += 5 * direction;
            target[kTilt1] += 12;
            target[kArmLeft] += 7;
            target[kArmRight] -= 7;
            break;
        case kAiAngry:
            add_pose(kAngry);
            target[kPan] += 6 * direction;
            target[kArmLeft] -= 12;
            target[kArmRight] += 12;
            break;
        case kAiCurious:
            target[kEyeLeft] -= 9 * direction;
            target[kEyeRight] -= 4 * direction;
            target[kPan] += 13 * direction;
            target[kTilt2] += phase == 0 ? 9 : 3;
            break;
        case kAiSurprised:
            target[kEyeLeft] -= 5 * direction;
            target[kEyeRight] += 5 * direction;
            target[kTilt2] += 12;
            target[kArmLeft] -= 18;
            target[kArmRight] += 18;
            break;
        default:
            target[kEyeLeft] -= 4 * direction;
            target[kEyeRight] += 4 * direction;
            target[kPan] += 7 * direction;
            target[kArmLeft] -= 5 * direction;
            target[kArmRight] += 5 * direction;
            break;
    }
    for (uint8_t id = 0; id < kServoCount; ++id)
        target[id] = ClampAngle(id, target[id]);
    return target;
}

bool MoveServosSmooth(const std::array<int, kServoCount>& target, int duration_ms,
                      bool require_speaking) {
    std::array<int, kServoCount> start{};
    {
        std::lock_guard<std::recursive_mutex> state_lock(g_state_mutex);
        for (uint8_t id = 0; id < kServoCount; ++id)
            start[id] = g_angles[id];
    }
    constexpr int kStepMs = 110;
    int largest_delta = 0;
    for (uint8_t id = 0; id < kServoCount; ++id)
        largest_delta = std::max(largest_delta, std::abs(target[id] - start[id]));
    // Never jump more than roughly two degrees per update. User-calibrated
    // poses with a large offset therefore take longer instead of snapping.
    const int steps = std::max({1, duration_ms / kStepMs, (largest_delta + 1) / 2});
    for (int step = 1; step <= steps; ++step) {
        if (g_server == nullptr || g_server->active() ||
            (require_speaking && !g_ai_speaking.load())) {
            return false;
        }
        for (uint8_t id = 0; id < kServoCount; ++id) {
            SetServo(id, start[id] + ((target[id] - start[id]) * step) / steps);
        }
        vTaskDelay(pdMS_TO_TICKS(kStepMs));
    }
    return true;
}

void GentleMotorWiggle(int emotion) {
    bool expected = false;
    if (!AiAnimationAllowed() || g_motors_running.load() ||
        !g_ai_motor_animation.compare_exchange_strong(expected, true))
        return;
    const int direction = emotion == kAiSad || emotion == kAiCurious ? -1 : 1;
    constexpr int kPowerSteps[] = {35, 45, 55, 63, 63, 55, 45, 35, 0};
    int outward_steps = 0;
    for (int power : kPowerSteps) {
        {
            std::lock_guard<std::recursive_mutex> motion_lock(g_motion_mutex);
            if (!AiAnimationAllowed() || !g_ai_motor_animation.load())
                break;
            Drive(direction * power, 0);
            // A second safety path guarantees that an interrupted animation cannot
            // leave either wheel energized.
            g_motion_deadline_us.store(esp_timer_get_time() + 250000);
        }
        ++outward_steps;
        vTaskDelay(pdMS_TO_TICKS(70));
    }
    // Replay exactly the completed outward ramp in reverse polarity. This
    // cancels most of the heading change instead of leaving Wall-E facing away.
    if (outward_steps > 0 && g_ai_motor_animation.load()) {
        StopMotors();
        vTaskDelay(pdMS_TO_TICKS(120));
        for (int step = outward_steps - 1; step >= 0; --step) {
            {
                std::lock_guard<std::recursive_mutex> motion_lock(g_motion_mutex);
                if (g_server == nullptr || g_server->active() || !g_ai_motor_animation.load()) {
                    break;
                }
                Drive(-direction * kPowerSteps[step], 0);
                g_motion_deadline_us.store(esp_timer_get_time() + 250000);
            }
            vTaskDelay(pdMS_TO_TICKS(70));
        }
    }
    if (g_ai_motor_animation.exchange(false))
        StopMotors();
}

void AiAnimationTask(void*) {
    bool was_speaking = false;
    bool motor_reacted = false;
    int phase = 0;
    while (true) {
        if (!AiAnimationAllowed()) {
            if (was_speaking) {
                if (g_ai_motor_animation.exchange(false))
                    StopMotors();
                MoveServosSmooth(HomeTargets(), 700, false);
                was_speaking = false;
                motor_reacted = false;
                phase = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        was_speaking = true;
        const int emotion = g_ai_emotion.load();
        if (!MoveServosSmooth(ReactionTargets(emotion, phase), 900, true))
            continue;
        if (!motor_reacted && AiAnimationAllowed()) {
            GentleMotorWiggle(emotion);
            motor_reacted = true;
        }
        phase ^= 1;
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void MotorSafetyTask(void*) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(25));
        if (!g_motors_running.load())
            continue;
        const int64_t now = esp_timer_get_time();
        const int64_t deadline = g_motion_deadline_us.load();
        const bool timed_motion_expired = deadline > 0 && now >= deadline;
        const bool manual_heartbeat_expired = deadline == 0 && g_server != nullptr &&
                                              g_server->active() &&
                                              now - g_last_drive_us.load() > 500000;
        if (timed_motion_expired || manual_heartbeat_expired)
            StopMotors();
    }
}

void ApplyPose(int pose) {
    std::lock_guard<std::recursive_mutex> state_lock(g_state_mutex);
    if (pose < 0 || pose >= kPoseCount)
        return;
    for (uint8_t id = 0; id < kServoCount; ++id)
        SetServo(id, g_settings.servo[id].home_angle + g_settings.pose_offsets[pose][id]);
}

char* ReadBody(httpd_req_t* req) {
    if (req->content_len <= 0 || req->content_len > 8192)
        return nullptr;
    char* data = static_cast<char*>(calloc(1, req->content_len + 1));
    if (!data)
        return nullptr;
    int got = 0;
    while (got < req->content_len) {
        const int count = httpd_req_recv(req, data + got, req->content_len - got);
        if (count <= 0) {
            free(data);
            return nullptr;
        }
        got += count;
    }
    return data;
}

esp_err_t SendJson(httpd_req_t* req, cJSON* root) {
    char* text = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!text)
        return ESP_ERR_NO_MEM;
    httpd_resp_set_type(req, "application/json");
    const esp_err_t err = httpd_resp_sendstr(req, text);
    free(text);
    return err;
}

void AddState(cJSON* root) {
    std::lock_guard<std::recursive_mutex> state_lock(g_state_mutex);
    cJSON_AddStringToObject(root, "ssid", kManualSsid);
    cJSON_AddStringToObject(root, "ip", "192.168.4.1");
    cJSON_AddBoolToObject(root, "camera", esp_camera_sensor_get() != nullptr);
    cJSON* servos = cJSON_AddArrayToObject(root, "servos");
    cJSON* angles = cJSON_AddArrayToObject(root, "angles");
    for (uint8_t id = 0; id < kServoCount; ++id) {
        cJSON* item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "name", kServoNames[id]);
        cJSON_AddNumberToObject(item, "channel", g_settings.servo[id].channel);
        cJSON_AddBoolToObject(item, "reverse", g_settings.servo[id].reverse);
        cJSON_AddNumberToObject(item, "min", g_settings.servo[id].pulse_min_us);
        cJSON_AddNumberToObject(item, "max", g_settings.servo[id].pulse_max_us);
        cJSON_AddNumberToObject(item, "home", g_settings.servo[id].home_angle);
        cJSON_AddItemToArray(servos, item);
        cJSON_AddItemToArray(angles, cJSON_CreateNumber(g_angles[id]));
    }
    cJSON* poses = cJSON_AddObjectToObject(root, "poses");
    for (uint8_t pose = 0; pose < kPoseCount; ++pose) {
        cJSON* values = cJSON_AddArrayToObject(poses, kPoseNames[pose]);
        for (uint8_t id = 0; id < kServoCount; ++id)
            cJSON_AddItemToArray(
                values, cJSON_CreateNumber(ClampAngle(id, g_settings.servo[id].home_angle +
                                                              g_settings.pose_offsets[pose][id])));
    }
    cJSON* motors = cJSON_AddObjectToObject(root, "motors");
    cJSON_AddBoolToObject(motors, "reverse_left", g_settings.motor_reverse_left);
    cJSON_AddBoolToObject(motors, "reverse_right", g_settings.motor_reverse_right);
}

esp_err_t Root(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, reinterpret_cast<const char*>(index_html_start),
                           index_html_end - index_html_start);
}
esp_err_t State(httpd_req_t* req) {
    cJSON* root = cJSON_CreateObject();
    AddState(root);
    return SendJson(req, root);
}
esp_err_t Stream(httpd_req_t* req) {
    constexpr int64_t kFramePeriodUs = 66667;  // Cap MJPEG at 15 FPS.
    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=wall-e-frame");
    char header[96];
    while (g_server != nullptr && g_server->active()) {
        const int64_t frame_started_us = esp_timer_get_time();
        std::unique_lock<std::mutex> camera_lock;
        if (g_camera_mutex != nullptr)
            camera_lock = std::unique_lock<std::mutex>(*g_camera_mutex);
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        const int length =
            snprintf(header, sizeof(header),
                     "\r\n--wall-e-frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                     static_cast<unsigned>(fb->len));
        esp_err_t err = httpd_resp_send_chunk(req, header, length);
        if (err == ESP_OK)
            err = httpd_resp_send_chunk(req, reinterpret_cast<const char*>(fb->buf), fb->len);
        esp_camera_fb_return(fb);
        if (camera_lock.owns_lock())
            camera_lock.unlock();
        if (err != ESP_OK)
            break;
        const int64_t remaining_us = kFramePeriodUs - (esp_timer_get_time() - frame_started_us);
        if (remaining_us > 1000) {
            vTaskDelay(pdMS_TO_TICKS((remaining_us + 999) / 1000));
        }
    }
    return ESP_OK;
}

esp_err_t Servo(httpd_req_t* req) {
    char* text = ReadBody(req);
    cJSON* json = text ? cJSON_Parse(text) : nullptr;
    free(text);
    cJSON* id = json ? cJSON_GetObjectItemCaseSensitive(json, "index") : nullptr;
    cJSON* angle = json ? cJSON_GetObjectItemCaseSensitive(json, "angle") : nullptr;
    if (!cJSON_IsNumber(id) || !cJSON_IsNumber(angle) || id->valueint < 0 ||
        id->valueint >= kServoCount) {
        if (json)
            cJSON_Delete(json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "servo");
    }
    SetServo(id->valueint, angle->valueint);
    cJSON_Delete(json);
    return httpd_resp_send(req, nullptr, 0);
}

esp_err_t DriveApi(httpd_req_t* req) {
    char* text = ReadBody(req);
    cJSON* json = text ? cJSON_Parse(text) : nullptr;
    free(text);
    cJSON* x = json ? cJSON_GetObjectItemCaseSensitive(json, "x") : nullptr;
    cJSON* y = json ? cJSON_GetObjectItemCaseSensitive(json, "y") : nullptr;
    if (!cJSON_IsNumber(x) || !cJSON_IsNumber(y)) {
        if (json)
            cJSON_Delete(json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "drive");
    }
    {
        std::lock_guard<std::recursive_mutex> motion_lock(g_motion_mutex);
        g_ai_motor_animation.store(false);
        Drive(x->valueint, y->valueint);
    }
    cJSON_Delete(json);
    return httpd_resp_send(req, nullptr, 0);
}

esp_err_t Neck(httpd_req_t* req) {
    char* text = ReadBody(req);
    cJSON* json = text ? cJSON_Parse(text) : nullptr;
    free(text);
    cJSON* x = json ? cJSON_GetObjectItemCaseSensitive(json, "x") : nullptr;
    cJSON* y = json ? cJSON_GetObjectItemCaseSensitive(json, "y") : nullptr;
    if (!cJSON_IsNumber(x) || !cJSON_IsNumber(y)) {
        if (json)
            cJSON_Delete(json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "neck");
    }
    std::lock_guard<std::recursive_mutex> state_lock(g_state_mutex);
    const int pan =
        g_settings.servo[kPan].home_angle - std::clamp(x->valueint, -100, 100) * 45 / 100;
    SetServo(kPan, pan);
    if (y->valueint < 0) {
        SetServo(kTilt1, 100 + (-std::clamp(y->valueint, -100, 0) * 80) / 100);
        SetServo(kTilt2, 112);
    } else {
        SetServo(kTilt1, 100);
        SetServo(kTilt2, 112 + (std::clamp(y->valueint, 0, 100) * 68) / 100);
    }
    cJSON_Delete(json);
    return httpd_resp_send(req, nullptr, 0);
}

esp_err_t MotorApi(httpd_req_t* req) {
    char* text = ReadBody(req);
    cJSON* json = text ? cJSON_Parse(text) : nullptr;
    free(text);
    cJSON* side = json ? cJSON_GetObjectItemCaseSensitive(json, "side") : nullptr;
    cJSON* power = json ? cJSON_GetObjectItemCaseSensitive(json, "power") : nullptr;
    const bool is_left = cJSON_IsString(side) && strcmp(side->valuestring, "left") == 0;
    const bool is_right = cJSON_IsString(side) && strcmp(side->valuestring, "right") == 0;
    if ((!is_left && !is_right) || !cJSON_IsNumber(power)) {
        if (json)
            cJSON_Delete(json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "motor");
    }
    {
        std::lock_guard<std::recursive_mutex> motion_lock(g_motion_mutex);
        g_ai_motor_animation.store(false);
        Motor(is_right, power->valueint);
    }
    g_last_drive_us.store(esp_timer_get_time());
    g_motion_deadline_us.store(0);
    g_motors_running.store(std::abs(power->valueint) > 2);
    cJSON_Delete(json);
    return httpd_resp_send(req, nullptr, 0);
}

int PoseFromName(const char* name) {
    for (int i = 0; i < kPoseCount; ++i)
        if (strcmp(name, kPoseNames[i]) == 0)
            return i;
    return -1;
}
esp_err_t Pose(httpd_req_t* req) {
    char* text = ReadBody(req);
    cJSON* json = text ? cJSON_Parse(text) : nullptr;
    free(text);
    cJSON* name = json ? cJSON_GetObjectItemCaseSensitive(json, "name") : nullptr;
    const int pose = cJSON_IsString(name) ? PoseFromName(name->valuestring) : -1;
    if (json)
        cJSON_Delete(json);
    if (pose < 0)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "pose");
    ApplyPose(pose);
    return httpd_resp_send(req, nullptr, 0);
}

bool ParseSettings(cJSON* json) {
    std::lock_guard<std::recursive_mutex> state_lock(g_state_mutex);
    PersistedSettings candidate = g_settings;
    cJSON* servos = cJSON_GetObjectItemCaseSensitive(json, "servos");
    if (cJSON_IsArray(servos))
        for (uint8_t id = 0; id < kServoCount && id < cJSON_GetArraySize(servos); ++id) {
            cJSON* value = cJSON_GetArrayItem(servos, id);
            if (!cJSON_IsObject(value))
                continue;
            cJSON* v = cJSON_GetObjectItemCaseSensitive(value, "channel");
            if (cJSON_IsNumber(v))
                candidate.servo[id].channel = std::clamp(v->valueint, 0, 15);
            v = cJSON_GetObjectItemCaseSensitive(value, "reverse");
            if (cJSON_IsBool(v))
                candidate.servo[id].reverse = cJSON_IsTrue(v);
            v = cJSON_GetObjectItemCaseSensitive(value, "min");
            if (cJSON_IsNumber(v))
                candidate.servo[id].pulse_min_us = std::clamp(v->valueint, 400, 2000);
            v = cJSON_GetObjectItemCaseSensitive(value, "max");
            if (cJSON_IsNumber(v))
                candidate.servo[id].pulse_max_us = std::clamp(v->valueint, 1000, 2800);
        }
    cJSON* poses = cJSON_GetObjectItemCaseSensitive(json, "poses");
    if (cJSON_IsObject(poses))
        for (uint8_t pose = 0; pose < kPoseCount; ++pose) {
            cJSON* values = cJSON_GetObjectItemCaseSensitive(poses, kPoseNames[pose]);
            if (!cJSON_IsArray(values))
                continue;
            for (uint8_t id = 0; id < kServoCount && id < cJSON_GetArraySize(values); ++id) {
                cJSON* v = cJSON_GetArrayItem(values, id);
                if (cJSON_IsNumber(v))
                    candidate.pose_offsets[pose][id] = std::clamp(
                        ClampAngle(id, v->valueint) - candidate.servo[id].home_angle, -90, 90);
            }
        }
    cJSON* motors = cJSON_GetObjectItemCaseSensitive(json, "motors");
    if (cJSON_IsObject(motors)) {
        cJSON* v = cJSON_GetObjectItemCaseSensitive(motors, "reverse_left");
        if (cJSON_IsBool(v))
            candidate.motor_reverse_left = cJSON_IsTrue(v);
        v = cJSON_GetObjectItemCaseSensitive(motors, "reverse_right");
        if (cJSON_IsBool(v))
            candidate.motor_reverse_right = cJSON_IsTrue(v);
    }
    if (!ValidateSettings(candidate))
        return false;
    g_settings = candidate;
    return true;
}
esp_err_t SettingsApi(httpd_req_t* req) {
    char* text = ReadBody(req);
    cJSON* json = text ? cJSON_Parse(text) : nullptr;
    free(text);
    if (!json || !ParseSettings(json)) {
        if (json)
            cJSON_Delete(json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid or conflicting settings");
    }
    cJSON_Delete(json);
    if (Save() != ESP_OK)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "save failed");
    return httpd_resp_send(req, nullptr, 0);
}

esp_err_t Home(httpd_req_t* req) {
    char* text = ReadBody(req);
    cJSON* json = text ? cJSON_Parse(text) : nullptr;
    free(text);
    cJSON* id = json ? cJSON_GetObjectItemCaseSensitive(json, "index") : nullptr;
    cJSON* angle = json ? cJSON_GetObjectItemCaseSensitive(json, "angle") : nullptr;
    if (!cJSON_IsNumber(id) || !cJSON_IsNumber(angle) || id->valueint < 0 ||
        id->valueint >= kServoCount) {
        if (json)
            cJSON_Delete(json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "home");
    }
    {
        std::lock_guard<std::recursive_mutex> state_lock(g_state_mutex);
        g_settings.servo[id->valueint].home_angle = ClampAngle(id->valueint, angle->valueint);
        SetServo(id->valueint, angle->valueint);
    }
    cJSON_Delete(json);
    if (Save() != ESP_OK)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "save failed");
    return httpd_resp_send(req, nullptr, 0);
}

esp_err_t ExitManual(httpd_req_t* req) {
    if (g_server != nullptr)
        g_server->RequestExit();
    return httpd_resp_sendstr(req, "Manual Control is closing");
}

bool EnableManualAccessPoint() {
    if (!g_ap_created) {
        esp_netif_t* ap_netif = esp_netif_create_default_wifi_ap();
        if (ap_netif == nullptr) {
            ESP_LOGE(kTag, "Could not create the Manual Control AP interface");
            return false;
        }
        g_ap_created = true;
    }

    wifi_config_t ap{};
    strcpy(reinterpret_cast<char*>(ap.ap.ssid), kManualSsid);
    strcpy(reinterpret_cast<char*>(ap.ap.password), kManualPassword);
    ap.ap.ssid_len = strlen(kManualSsid);
    ap.ap.channel = 1;
    ap.ap.max_connection = 4;
    ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap.ap.ssid_hidden = 0;
    ap.ap.beacon_interval = 100;

    // set_mode() dynamically starts the AP control block while preserving the
    // existing Xiaozhi station connection. Reapply this on every voice command
    // so a later network event cannot leave active_ true with the AP disabled.
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "Failed to enable APSTA mode: %s", esp_err_to_name(err));
        return false;
    }
    err = esp_wifi_set_config(WIFI_IF_AP, &ap);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "Failed to configure Manual Wi-Fi: %s", esp_err_to_name(err));
        return false;
    }

    // esp_wifi_start() is safe after mode/config changes and makes the intent
    // explicit on boards whose station connection was started earlier.
    err = esp_wifi_start();
    if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_STOPPED) {
        ESP_LOGE(kTag, "Failed to start Manual Wi-Fi: %s", esp_err_to_name(err));
        return false;
    }

    wifi_mode_t mode = WIFI_MODE_NULL;
    wifi_config_t applied{};
    if (esp_wifi_get_mode(&mode) != ESP_OK || mode != WIFI_MODE_APSTA ||
        esp_wifi_get_config(WIFI_IF_AP, &applied) != ESP_OK) {
        ESP_LOGE(kTag, "Manual Wi-Fi verification failed (mode=%d)", static_cast<int>(mode));
        return false;
    }
    ESP_LOGI(kTag, "Manual Wi-Fi active: SSID=%s, IP=192.168.4.1, mode=APSTA",
             reinterpret_cast<const char*>(applied.ap.ssid));
    return true;
}

bool AddHandler(httpd_handle_t server, const char* uri, httpd_method_t method,
                esp_err_t (*handler)(httpd_req_t*)) {
    httpd_uri_t entry = {.uri = uri, .method = method, .handler = handler};
    const esp_err_t err = httpd_register_uri_handler(server, &entry);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "Could not register %s: %s", uri, esp_err_to_name(err));
        return false;
    }
    return true;
}

void StopServers() {
    if (g_stream != nullptr) {
        httpd_stop(g_stream);
        g_stream = nullptr;
    }
    if (g_http != nullptr) {
        httpd_stop(g_http);
        g_http = nullptr;
    }
}
}  // namespace

ManualControlServer::ManualControlServer(PwmWriter write_pwm, std::mutex* camera_mutex,
                                         ExitCallback exit_callback)
    : write_pwm_(std::move(write_pwm)),
      camera_mutex_(camera_mutex),
      exit_callback_(std::move(exit_callback)) {}

void ManualControlServer::Initialize() {
    if (initialized_.exchange(true))
        return;
    g_server = this;
    g_write_pwm = write_pwm_;
    g_camera_mutex = camera_mutex_;
    Load();
    for (uint8_t id = 0; id < kServoCount; ++id)
        SetServo(id, g_settings.servo[id].home_angle);
    StopMotors();
    if (g_motor_safety_task == nullptr) {
        if (xTaskCreate(MotorSafetyTask, "wall_e_motor_safe", 3072, nullptr, 3,
                        &g_motor_safety_task) != pdPASS) {
            g_motor_safety_task = nullptr;
            ESP_LOGE(kTag, "Could not create the motor safety task");
        }
    }
    if (g_ai_animation_task == nullptr) {
        if (xTaskCreate(AiAnimationTask, "wall_e_ai_move", 4096, nullptr, 2,
                        &g_ai_animation_task) != pdPASS) {
            g_ai_animation_task = nullptr;
            ESP_LOGE(kTag, "Could not create the AI animation task");
        }
    }
}

bool ManualControlServer::Start() {
    if (!initialized_.load())
        Initialize();
    if (g_motor_safety_task == nullptr)
        return false;
    g_ai_speaking.store(false);
    g_ai_motor_animation.store(false);
    StopMotors();
    if (!EnableManualAccessPoint())
        return false;
    if (active_.load()) {
        esp_wifi_set_ps(WIFI_PS_NONE);
        ESP_LOGI(kTag, "Manual Control was already active; Wi-Fi AP was refreshed");
        return true;
    }
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 6144;
    config.task_priority = 8;
    config.max_uri_handlers = 12;
    config.lru_purge_enable = true;
    if (httpd_start(&g_http, &config) != ESP_OK) {
        Stop();
        return false;
    }
    const bool control_handlers_ok = AddHandler(g_http, "/", HTTP_GET, Root) &&
                                     AddHandler(g_http, "/api/state", HTTP_GET, State) &&
                                     AddHandler(g_http, "/api/servo", HTTP_POST, Servo) &&
                                     AddHandler(g_http, "/api/drive", HTTP_POST, DriveApi) &&
                                     AddHandler(g_http, "/api/neck", HTTP_POST, Neck) &&
                                     AddHandler(g_http, "/api/motor", HTTP_POST, MotorApi) &&
                                     AddHandler(g_http, "/api/pose", HTTP_POST, Pose) &&
                                     AddHandler(g_http, "/api/settings", HTTP_POST, SettingsApi) &&
                                     AddHandler(g_http, "/api/home", HTTP_POST, Home) &&
                                     AddHandler(g_http, "/api/exit", HTTP_POST, ExitManual);
    if (!control_handlers_ok) {
        Stop();
        return false;
    }
    config.server_port = 81;
    config.ctrl_port = 32769;
    config.stack_size = 4096;
    config.task_priority = 3;
    config.max_uri_handlers = 2;
    config.recv_wait_timeout = 1;
    config.send_wait_timeout = 1;
    if (httpd_start(&g_stream, &config) != ESP_OK ||
        !AddHandler(g_stream, "/stream", HTTP_GET, Stream)) {
        Stop();
        return false;
    }
    g_previous_wifi_ps_valid = esp_wifi_get_ps(&g_previous_wifi_ps) == ESP_OK;
    if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK)
        ESP_LOGW(kTag, "Could not disable Wi-Fi power save");
    active_.store(true);
    for (uint8_t id = 0; id < kServoCount; ++id)
        SetServo(id, g_settings.servo[id].home_angle);
    StopMotors();
    g_last_drive_us.store(esp_timer_get_time());
    ESP_LOGI(kTag, "Manual Control is live at http://192.168.4.1");
    return true;
}

void ManualControlServer::Stop() {
    active_.store(false);
    StopMotors();
    StopServers();
    const esp_err_t mode_err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (mode_err != ESP_OK)
        ESP_LOGW(kTag, "Could not disable Manual Wi-Fi: %s", esp_err_to_name(mode_err));
    if (g_previous_wifi_ps_valid) {
        esp_wifi_set_ps(g_previous_wifi_ps);
        g_previous_wifi_ps_valid = false;
    }
}

bool ManualControlServer::DriveFor(const std::string& direction, int duration_ms) {
    int x = 0;
    int y = 0;
    if (direction == "forward")
        y = 100;
    else if (direction == "backward")
        y = -100;
    else if (direction == "left")
        x = -100;
    else if (direction == "right")
        x = 100;
    else
        return false;
    {
        std::lock_guard<std::recursive_mutex> motion_lock(g_motion_mutex);
        g_ai_motor_animation.store(false);
        Drive(x, y);
    }
    g_motion_deadline_us.store(esp_timer_get_time() +
                               static_cast<int64_t>(std::clamp(duration_ms, 100, 5000)) * 1000);
    g_motors_running.store(true);
    return true;
}

void ManualControlServer::RequestExit() {
    if (exit_callback_)
        exit_callback_();
}

bool ManualControlServer::SetServoAngle(int index, int angle) {
    if (index < 0 || index >= kServoCount)
        return false;
    SetServo(static_cast<uint8_t>(index), angle);
    return true;
}

bool ManualControlServer::ApplyNamedPose(const std::string& pose) {
    const int id = PoseFromName(pose.c_str());
    if (id < 0)
        return false;
    ApplyPose(id);
    return true;
}

void ManualControlServer::SetAiSpeaking(bool speaking) {
    if (active_.load() && speaking)
        return;
    g_ai_speaking.store(speaking);
}

void ManualControlServer::SetAiEmotion(const std::string& emotion) {
    std::string normalized = emotion;
    for (char& character : normalized) {
        if (character >= 'A' && character <= 'Z')
            character = static_cast<char>(character - 'A' + 'a');
    }
    int mapped = kAiNeutral;
    if (normalized == "happy" || normalized == "laughing" || normalized == "funny" ||
        normalized == "loving" || normalized == "kissy" || normalized == "confident") {
        mapped = kAiHappy;
    } else if (normalized == "sad" || normalized == "crying" || normalized == "sleepy") {
        mapped = kAiSad;
    } else if (normalized == "angry" || normalized == "disgusted") {
        mapped = kAiAngry;
    } else if (normalized == "thinking" || normalized == "confused" || normalized == "suspicious" ||
               normalized == "embarrassed") {
        mapped = kAiCurious;
    } else if (normalized == "surprised" || normalized == "shocked" || normalized == "nervous" ||
               normalized == "silly") {
        mapped = kAiSurprised;
    }
    g_ai_emotion.store(mapped);
}

void ManualControlServer::PlayBootAnimation() {
    const BaseType_t task_result = xTaskCreate(
        [](void*) {
            int eye_left;
            int eye_right;
            int tilt1;
            int tilt2;
            int arm_left;
            int arm_right;
            {
                std::lock_guard<std::recursive_mutex> state_lock(g_state_mutex);
                eye_left = g_settings.servo[kEyeLeft].home_angle;
                eye_right = g_settings.servo[kEyeRight].home_angle;
                tilt1 = g_settings.servo[kTilt1].home_angle;
                tilt2 = g_settings.servo[kTilt2].home_angle;
                arm_left = g_settings.servo[kArmLeft].home_angle;
                arm_right = g_settings.servo[kArmRight].home_angle;
            }

            // The supplied Wall-E voice is about 2.04 seconds long. Start just
            // after the audio begins, then spread the three poses over 1.85 seconds
            // so the return to Home lands close to the end of the voice.
            vTaskDelay(pdMS_TO_TICKS(80));
            SetServo(kEyeLeft, eye_left - 10);
            SetServo(kEyeRight, eye_right + 10);
            SetServo(kArmLeft, arm_left - 18);
            SetServo(kArmRight, arm_right + 18);
            SetServo(kTilt1, tilt1 + 10);
            vTaskDelay(pdMS_TO_TICKS(600));

            SetServo(kEyeLeft, eye_left + 10);
            SetServo(kEyeRight, eye_right - 10);
            SetServo(kArmLeft, arm_left + 12);
            SetServo(kArmRight, arm_right - 12);
            SetServo(kTilt1, tilt1);
            SetServo(kTilt2, tilt2 + 10);
            vTaskDelay(pdMS_TO_TICKS(650));

            SetServo(kArmLeft, arm_left - 12);
            SetServo(kArmRight, arm_right + 12);
            SetServo(kTilt2, tilt2);
            SetServo(kTilt1, tilt1 + 8);
            vTaskDelay(pdMS_TO_TICKS(600));

            {
                std::lock_guard<std::recursive_mutex> state_lock(g_state_mutex);
                for (uint8_t id = 0; id < kServoCount; ++id) {
                    SetServo(id, g_settings.servo[id].home_angle);
                }
            }
            vTaskDelete(nullptr);
        },
        "wall_e_boot_move", 3072, nullptr, 2, nullptr);
    if (task_result != pdPASS)
        ESP_LOGW(kTag, "Could not create the boot animation task");
}
