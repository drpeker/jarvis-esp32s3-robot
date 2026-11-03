#include "motor_controller.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>

#include <esp_log.h>
#include <esp_timer.h>

#include "config.h"
#include "mcp_server.h"
#include "mpu6050.h"
#include "settings.h"

#define TAG "MochanMotor"

namespace {
constexpr ledc_mode_t kSpeedMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kTimer = LEDC_TIMER_2;
constexpr ledc_timer_bit_t kDutyResolution = LEDC_TIMER_10_BIT;
constexpr uint32_t kMaxDuty = (1U << 10) - 1;
constexpr char kSettingsNamespace[] = "robot";
constexpr char kSpeedFactorKey[] = "speed_factor";
constexpr char kTurnMsPerDegreeKey[] = "turn_ms_deg";
constexpr const char* kMinDutyKeys[2][2] = {{"min_lf", "min_lr"}, {"min_rf", "min_rr"}};
}  // namespace

MotorController::MotorController(gpio_num_t left_in1, gpio_num_t left_in2, gpio_num_t right_in1,
                                 gpio_num_t right_in2)
    : pins_{left_in1, left_in2, right_in1, right_in2} {
    LoadSettings();
    ConfigurePwm();
    ApplySpeeds(0, 0);

    // Commands run one after another (a "go, turn, go" sequence from the AI); only Stop()
    // clears the queue and interrupts the running command.
    command_queue_ = xQueueCreate(8, sizeof(Command));
    if (command_queue_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create command queue");
        abort();
    }

    if (xTaskCreate(TaskEntry, "mochan_motor", 4096, this, 4, &task_handle_) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create motor task");
        abort();
    }

    RegisterMcpTools();
}

MotorController::~MotorController() {
    ApplySpeeds(0, 0);
    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
    }
    if (command_queue_ != nullptr) {
        vQueueDelete(command_queue_);
    }
}

void MotorController::ConfigurePwm() {
    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode = kSpeedMode;
    timer_config.duty_resolution = kDutyResolution;
    timer_config.timer_num = kTimer;
    timer_config.freq_hz = tuning_.pwm_hz;
    timer_config.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    for (size_t i = 0; i < 4; ++i) {
        ledc_channel_config_t channel_config = {};
        channel_config.gpio_num = pins_[i];
        channel_config.speed_mode = kSpeedMode;
        channel_config.channel = channels_[i];
        channel_config.intr_type = LEDC_INTR_DISABLE;
        channel_config.timer_sel = kTimer;
        channel_config.duty = 0;
        channel_config.hpoint = 0;
        channel_config.flags.output_invert = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
    }
}

bool MotorController::Drive(int left_speed, int right_speed, uint32_t duration_ms,
                            int target_degrees, int target_cm) {
    if ((motion_locked_.load() || raw_mode_.load()) && (left_speed != 0 || right_speed != 0)) {
        return false;
    }
    // The dead band is handled per motor in SetMotor, so only the range is limited here.
    auto apply_start_threshold = [](int speed) { return std::clamp(speed, -100, 100); };

    Command command = {
        .left_speed = apply_start_threshold(left_speed),
        .right_speed = apply_start_threshold(right_speed),
        .duration_ms = std::clamp(duration_ms, uint32_t{100}, uint32_t{MOTOR_MAX_RUN_TIME_MS}),
        .target_degrees = target_degrees,
        .target_cm = target_cm,
    };
    return xQueueSend(command_queue_, &command, 0) == pdPASS;
}

void MotorController::Stop() {
    Command command = {};
    xQueueReset(command_queue_);
    xQueueSend(command_queue_, &command, 0);
}

bool MotorController::IsMoving() const { return moving_.load(); }

int MotorController::LeftSpeed() const { return left_speed_.load(); }

int MotorController::RightSpeed() const { return right_speed_.load(); }

int MotorController::SpeedFactor() const { return speed_factor_.load(); }

int MotorController::TurnMsPerDegree() const { return turn_ms_per_degree_.load(); }

void MotorController::SetMotionLocked(bool locked) {
    motion_locked_.store(locked);
    if (locked) {
        Stop();
    }
}

bool MotorController::IsMotionLocked() const { return motion_locked_.load(); }

void MotorController::LoadSettings() {
    Settings settings(kSettingsNamespace, false);
    speed_factor_.store(std::clamp(
        static_cast<int>(settings.GetInt(kSpeedFactorKey, MOTOR_SPEED_FACTOR_DEFAULT)),
        MOTOR_SPEED_FACTOR_MIN, MOTOR_SPEED_FACTOR_MAX));
    turn_ms_per_degree_.store(std::clamp(
        static_cast<int>(settings.GetInt(kTurnMsPerDegreeKey, MOTOR_TURN_MS_PER_DEGREE_DEFAULT)),
        1, MOTOR_TURN_MS_PER_DEGREE_MAX));
    for (int m = 0; m < 2; ++m) {
        for (int d = 0; d < 2; ++d) {
            tuning_.min_duty[m][d] = settings.GetInt(kMinDutyKeys[m][d], MOTOR_MIN_DUTY_PERCENT);
        }
    }
    tuning_.gain[0] = settings.GetInt("gain_l", 100);
    tuning_.gain[1] = settings.GetInt("gain_r", 100);
    tuning_.turn_speed = settings.GetInt("turn_spd", MOTOR_TURN_SPEED_PERCENT);
    tuning_.turn_lead_x10 = settings.GetInt("lead_x10", MOTOR_TURN_STOP_LEAD_DEGREES * 10);
    tuning_.kick_ms = settings.GetInt("kick_ms", MOTOR_KICK_START_MS);
    tuning_.brake_ms = settings.GetInt("brake_ms", MOTOR_TURN_BRAKE_MS);
    tuning_.pwm_hz = settings.GetInt("pwm_hz", MOTOR_PWM_FREQUENCY_HZ);
    tuning_.accel_axis = settings.GetInt("acc_axis", 0);
    tuning_.accel_sign = settings.GetInt("acc_sign", 1);
    tuning_.dist_speed = settings.GetInt("dist_spd", MOTOR_DIST_SPEED_PERCENT);
    tuning_.ms20[0] = settings.GetInt("ms20_fwd", MOTOR_DIST_MS20_FWD);
    tuning_.ms20[1] = settings.GetInt("ms20_back", MOTOR_DIST_MS20_BACK);
    ESP_LOGI(TAG,
             "Speed factor %d%%, turn %d ms/deg; min duty L %d/%d R %d/%d, gain L %d R %d, "
             "turn %d%%, lead %.1f, kick %d ms, brake %d ms, PWM %d Hz",
             speed_factor_.load(), turn_ms_per_degree_.load(), tuning_.min_duty[0][0],
             tuning_.min_duty[0][1], tuning_.min_duty[1][0], tuning_.min_duty[1][1],
             tuning_.gain[0], tuning_.gain[1], tuning_.turn_speed,
             tuning_.turn_lead_x10 / 10.0f, tuning_.kick_ms, tuning_.brake_ms, tuning_.pwm_hz);
}

MotorController::Tuning MotorController::GetTuning() const { return tuning_; }

void MotorController::SetTuning(const Tuning& tuning, bool save) {
    const int old_pwm_hz = tuning_.pwm_hz;
    tuning_ = tuning;
    for (auto& motor : tuning_.min_duty) {
        for (int& duty : motor) {
            duty = std::clamp(duty, 0, 95);
        }
    }
    for (int& gain : tuning_.gain) {
        gain = std::clamp(gain, 30, 100);
    }
    tuning_.turn_speed = std::clamp(tuning_.turn_speed, 10, 100);
    tuning_.turn_lead_x10 = std::clamp(tuning_.turn_lead_x10, 0, 300);
    tuning_.kick_ms = std::clamp(tuning_.kick_ms, 0, 300);
    tuning_.brake_ms = std::clamp(tuning_.brake_ms, 0, 300);
    tuning_.pwm_hz = std::clamp(tuning_.pwm_hz, 100, 20000);
    tuning_.accel_axis = tuning_.accel_axis == 1 ? 1 : 0;
    tuning_.accel_sign = tuning_.accel_sign < 0 ? -1 : 1;
    tuning_.dist_speed = std::clamp(tuning_.dist_speed, 1, 100);
    for (int& ms : tuning_.ms20) {
        ms = std::clamp(ms, 100, 5000);
    }
    if (tuning_.pwm_hz != old_pwm_hz) {
        ledc_set_freq(kSpeedMode, kTimer, tuning_.pwm_hz);
    }
    if (!save) {
        return;
    }
    Settings settings(kSettingsNamespace, true);
    for (int m = 0; m < 2; ++m) {
        for (int d = 0; d < 2; ++d) {
            settings.SetInt(kMinDutyKeys[m][d], tuning_.min_duty[m][d]);
        }
    }
    settings.SetInt("gain_l", tuning_.gain[0]);
    settings.SetInt("gain_r", tuning_.gain[1]);
    settings.SetInt("turn_spd", tuning_.turn_speed);
    settings.SetInt("lead_x10", tuning_.turn_lead_x10);
    settings.SetInt("kick_ms", tuning_.kick_ms);
    settings.SetInt("brake_ms", tuning_.brake_ms);
    settings.SetInt("pwm_hz", tuning_.pwm_hz);
    settings.SetInt("acc_axis", tuning_.accel_axis);
    settings.SetInt("acc_sign", tuning_.accel_sign);
    settings.SetInt("dist_spd", tuning_.dist_speed);
    settings.SetInt("ms20_fwd", tuning_.ms20[0]);
    settings.SetInt("ms20_back", tuning_.ms20[1]);
    ESP_LOGI(TAG, "Motor tuning saved");
}

void MotorController::SetRawMode(bool on) {
    if (on) {
        raw_mode_.store(true);
        Stop();
        vTaskDelay(pdMS_TO_TICKS(150));
    } else {
        SetRawDuty(0, 0);
        raw_mode_.store(false);
    }
}

void MotorController::SetRawDuty(int left_percent, int right_percent) {
    if (!raw_mode_.load()) {
        return;
    }
    SetMotor(left_percent, 0, true);
    SetMotor(right_percent, 1, true);
    moving_.store(left_percent != 0 || right_percent != 0);
}

void MotorController::SaveSetting(const char* key, int value) {
    Settings settings(kSettingsNamespace, true);
    settings.SetInt(key, value);
}

void MotorController::TaskEntry(void* arg) { static_cast<MotorController*>(arg)->TaskLoop(); }

void MotorController::TaskLoop() {
    bool pending = false;
    Command command;

    while (true) {
        if (!pending && xQueueReceive(command_queue_, &command, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        pending = false;

        if (command.left_speed == 0 && command.right_speed == 0) {
            ApplySpeeds(0, 0);
            continue;
        }
        if (command.target_degrees > 0 && gyro_ != nullptr) {
            pending = RunGyroTurn(command, command);
            continue;
        }
        // Timed move: runs to the end unless a stop arrives; queued moves wait their turn.
        ApplySpeeds(command.left_speed, command.right_speed);
        const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(command.duration_ms);
        while (static_cast<int32_t>(deadline - xTaskGetTickCount()) > 0) {
            if (StopRequested(command, 1)) {
                pending = true;
                break;
            }
        }
        if (!pending) {
            if (command.target_cm > 0) {
                Brake();  // distance moves stop the same way every time
            } else {
                ApplySpeeds(0, 0);
            }
        }
    }
}

uint32_t MotorController::DistanceMs(int cm, bool backward) const {
    return static_cast<uint32_t>(std::max(cm, 0)) * tuning_.ms20[backward ? 1 : 0] / 20;
}

// True (and the stop command in `next`) when Stop() was called; otherwise waits `wait`.
bool MotorController::StopRequested(Command& next, TickType_t wait) {
    if (xQueuePeek(command_queue_, &next, 0) == pdTRUE && next.left_speed == 0 &&
        next.right_speed == 0) {
        xQueueReceive(command_queue_, &next, 0);
        return true;
    }
    vTaskDelay(wait);
    return false;
}

// Closed-loop turn in place. The yaw is integrated without a gap from the moment the
// motors start, through braking and settling: the robot turns toward the requested side
// until the measured angle reaches the target, brakes, measures where it really stopped
// and, if it is off by more than kTurnToleranceDeg, corrects toward the target (forward
// or back) up to kMaxCorrections times. Left turns are counter-clockwise (+yaw).
// Returns true when a new command interrupted the turn (it is left in `next`).
bool MotorController::RunGyroTurn(Command turn, Command& next) {
    constexpr float kTurnToleranceDeg = 3.0f;
    constexpr int kMaxCorrections = 4;
    const TickType_t poll_ticks = std::max<TickType_t>(1, pdMS_TO_TICKS(MOTOR_TURN_SAMPLE_MS));
    const float target = static_cast<float>(turn.target_degrees);
    const float side = turn.left_speed < 0 ? 1.0f : -1.0f;  // +1 left (CCW), -1 right
    const int speed = std::max(std::abs(turn.left_speed), std::abs(turn.right_speed));
    // Stop early by the coast measured on the previous turns (braking from ~100 deg/s
    // still carries the body several degrees).
    const float lead = coast_lead_;

    // Signed yaw since the start, from the gap-free 200 Hz FIFO heading (the polling
    // fallback is only used if the FIFO could not be started).
    float angle = 0.0f;
    const float start_heading = gyro_->Heading();
    int64_t last_us = esp_timer_get_time();
    auto sample = [&]() {
        const int64_t now_us = esp_timer_get_time();
        if (gyro_->HeadingOk()) {
            angle = gyro_->Heading() - start_heading;
        } else {
            float dps = 0.0f;
            if (gyro_->ReadYawRate(dps)) {
                angle += dps * static_cast<float>(now_us - last_us) / 1e6f;
            }
        }
        last_us = now_us;
    };
    auto progress = [&]() { return angle * side; };
    // Wait until `until_us` while integrating; false when a new command arrived.
    auto wait_until = [&](int64_t until_us) {
        while (esp_timer_get_time() < until_us) {
            if (StopRequested(next, poll_ticks)) {
                return false;
            }
            sample();
        }
        return true;
    };
    // toward = +1 turns toward the target side, -1 back.
    auto spin = [&](float toward) {
        const int s = static_cast<int>(side * toward) * speed;
        SetSpeedsNow(-s, s);  // left turn: left track back, right track forward
    };
    auto brake_and_settle = [&]() {
        for (size_t i = 0; i < 4; ++i) {
            SetDuty(channels_[i], kMaxDuty);
        }
        const bool ok = wait_until(esp_timer_get_time() + tuning_.brake_ms * 1000LL);
        SetSpeedsNow(0, 0);
        return ok && wait_until(esp_timer_get_time() + MOTOR_TURN_SETTLE_MS * 1000LL);
    };
    auto interrupted = [&]() {
        SetSpeedsNow(0, 0);
        ESP_LOGI(TAG, "Gyro turn interrupted at %.1f deg", progress());
        return true;
    };

    // 1) Turn toward the target until the measured angle reaches it. Safety: never spin
    // longer than the angle needs at a slow 45 deg/s (+0.6 s to get going).
    const int64_t max_turn_ms =
        std::min<int64_t>(turn.duration_ms, 600 + static_cast<int64_t>(target * 1000 / 45));
    const int64_t deadline_us = esp_timer_get_time() + max_turn_ms * 1000;
    last_us = esp_timer_get_time();
    spin(1);
    bool reached = false;
    while (true) {
        if (StopRequested(next, poll_ticks)) {
            return interrupted();
        }
        sample();
        if (progress() >= target - lead) {
            reached = true;
            break;
        }
        if (esp_timer_get_time() >= deadline_us) {
            break;
        }
    }
    const float at_stop = progress();
    if (!brake_and_settle()) {
        return interrupted();
    }
    const float first_final = progress();

    // Learn the coast for the next turn from what really happened after the stop.
    if (reached) {
        const float coast = first_final - at_stop;
        coast_lead_ = std::clamp(0.6f * coast_lead_ + 0.4f * coast, 0.0f, 25.0f);
    }

    // 2) Check where it really stopped; correct with short pushes, measuring after each.
    int corrections = 0;
    while (reached && corrections < kMaxCorrections) {
        const float error = target - progress();
        if (std::fabs(error) <= kTurnToleranceDeg) {
            break;
        }
        ++corrections;
        const float toward = error > 0 ? 1.0f : -1.0f;
        const int pulse_ms = std::clamp(static_cast<int>(std::fabs(error) * 5), 40, 160);
        spin(toward);
        if (!wait_until(esp_timer_get_time() + pulse_ms * 1000LL) || !brake_and_settle()) {
            return interrupted();
        }
    }

    const float final_angle = progress();
    last_turn_final_x10_.store(static_cast<int>(std::lround(final_angle * 10)));
    turn_count_.fetch_add(1);
    if (reached) {
        ESP_LOGI(TAG,
                 "Gyro turn: target=%d lead=%.1f at_stop=%.1f after_brake=%.1f corrections=%d "
                 "final=%.1f deg (next lead %.1f)",
                 turn.target_degrees, lead, at_stop, first_final, corrections, final_angle,
                 coast_lead_);
    } else {
        ESP_LOGW(TAG, "Gyro turn timed out: target=%d reached only %.1f deg",
                 turn.target_degrees, final_angle);
    }
    return false;
}

// Sets both tracks at once (no kick, no stagger, no dead time) so that a caller which is
// integrating the gyro does not lose time inside ApplySpeeds.
void MotorController::SetSpeedsNow(int left_speed, int right_speed) {
    const int physical_left = MOTOR_LEFT_INVERT ? -left_speed : left_speed;
    const int physical_right = MOTOR_RIGHT_INVERT ? -right_speed : right_speed;
    SetMotor(physical_left, 0);
    SetMotor(physical_right, 1);
    last_physical_left_speed_ = physical_left;
    last_physical_right_speed_ = physical_right;
    left_speed_.store(left_speed);
    right_speed_.store(right_speed);
    moving_.store(left_speed != 0 || right_speed != 0);
}

// MX1508 brake: both inputs high shorts the motor, stopping it faster than coasting.
void MotorController::Brake() {
    for (size_t i = 0; i < 4; ++i) {
        SetDuty(channels_[i], kMaxDuty);
    }
    vTaskDelay(pdMS_TO_TICKS(tuning_.brake_ms));
    ApplySpeeds(0, 0);
}

void MotorController::ApplySpeeds(int left_speed, int right_speed) {
    left_speed = std::clamp(left_speed, -100, 100);
    right_speed = std::clamp(right_speed, -100, 100);

    int physical_left_speed = left_speed;
    int physical_right_speed = right_speed;
    if (MOTOR_LEFT_INVERT) {
        physical_left_speed = -physical_left_speed;
    }
    if (MOTOR_RIGHT_INVERT) {
        physical_right_speed = -physical_right_speed;
    }

    const bool reversing =
        (last_physical_left_speed_ != 0 && physical_left_speed != 0 &&
         ((last_physical_left_speed_ > 0) != (physical_left_speed > 0))) ||
        (last_physical_right_speed_ != 0 && physical_right_speed != 0 &&
         ((last_physical_right_speed_ > 0) != (physical_right_speed > 0)));

    if (reversing) {
        // Coast both motors before changing polarity. This avoids a full-power
        // shoot-through/current spike that can brown out or desynchronise the camera.
        SetMotor(0, 0);
        SetMotor(0, 1);
        last_physical_left_speed_ = 0;
        last_physical_right_speed_ = 0;
        vTaskDelay(pdMS_TO_TICKS(MOTOR_DIRECTION_DEAD_TIME_MS));
    }

    const bool needs_kick = tuning_.kick_ms > 0 && (
        (last_physical_left_speed_ == 0 && physical_left_speed != 0 &&
         std::abs(physical_left_speed) < 100) ||
        (last_physical_right_speed_ == 0 && physical_right_speed != 0 &&
         std::abs(physical_right_speed) < 100));
    if (needs_kick) {
        // Low duty may not overcome static friction; pulse full power briefly first.
        auto full = [](int speed) { return speed > 0 ? 100 : (speed < 0 ? -100 : 0); };
        SetMotor(full(physical_left_speed), 0, true);
        if (physical_left_speed != 0 && physical_right_speed != 0) {
            // Same inrush staggering as a full-power start.
            vTaskDelay(pdMS_TO_TICKS(MOTOR_DUAL_START_STAGGER_MS));
        }
        SetMotor(full(physical_right_speed), 1, true);
        vTaskDelay(pdMS_TO_TICKS(tuning_.kick_ms));
    }

    const bool starting_both = !needs_kick && last_physical_left_speed_ == 0 &&
                               last_physical_right_speed_ == 0 &&
                               physical_left_speed != 0 && physical_right_speed != 0;
    SetMotor(physical_left_speed, 0);
    if (starting_both) {
        // Preserve the requested 100% running power while reducing simultaneous inrush.
        vTaskDelay(pdMS_TO_TICKS(MOTOR_DUAL_START_STAGGER_MS));
    }
    SetMotor(physical_right_speed, 1);

    last_physical_left_speed_ = physical_left_speed;
    last_physical_right_speed_ = physical_right_speed;
    left_speed_.store(left_speed);
    right_speed_.store(right_speed);
    moving_.store(left_speed != 0 || right_speed != 0);
}

void MotorController::SetMotor(int speed, int motor, bool raw) {
    const ledc_channel_t forward_channel = channels_[motor * 2];
    const ledc_channel_t reverse_channel = channels_[motor * 2 + 1];
    int percent = std::min(std::abs(speed), 100);
    if (!raw && percent > 0) {
        // Logical 1..100 -> calibrated dead band .. 100%, scaled by the motor's gain.
        const int min_duty = tuning_.min_duty[motor][speed > 0 ? 0 : 1];
        const int scaled = percent * tuning_.gain[motor] / 100;
        percent = min_duty + (100 - min_duty) * std::max(scaled, 1) / 100;
    }
    uint32_t duty = static_cast<uint32_t>(percent) * MOTOR_MAX_DUTY_PERCENT * kMaxDuty / 10000U;

    if (speed > 0) {
        SetDuty(reverse_channel, 0);
        SetDuty(forward_channel, duty);
    } else if (speed < 0) {
        SetDuty(forward_channel, 0);
        SetDuty(reverse_channel, duty);
    } else {
        SetDuty(forward_channel, 0);
        SetDuty(reverse_channel, 0);
    }
}

void MotorController::SetDuty(ledc_channel_t channel, uint32_t duty) {
    ESP_ERROR_CHECK(ledc_set_duty(kSpeedMode, channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(kSpeedMode, channel));
}

void MotorController::RegisterMcpTools() {
    auto& mcp_server = McpServer::GetInstance();

    mcp_server.AddTool(
        "self.robot.drive",
        "Move the robot's two-wheel base. This robot's name is Jarvis; when speaking about "
        "itself, always call itself Jarvis. direction must be forward, backward, left, or "
        "right; left/right mean turning the whole robot in place using both motors. Never use "
        "this tool when the user asks for only one motor; use self.robot.run_motor instead. "
        "forward/backward: PWM = speed x the stored speed factor (see "
        "self.robot.set_speed_factor); duration_ms is limited for safety. left/right: give "
        "the angle in degrees; if the user does not say an angle, omit degrees and the robot "
        "turns 20 degrees. duration_ms is ignored for turns. forward/backward over a distance: "
        "give distance_cm and omit speed and duration_ms; the move is timed at a fixed "
        "calibrated speed (about 20 cm per second, at most 5 s per call). If the user gives "
        "neither a distance nor a time for forward/backward, omit both: Jarvis goes 10 cm.",
        PropertyList({
            Property("direction", kPropertyTypeString, "forward"),
            Property("speed", kPropertyTypeInteger, 100, 0, 100),
            Property("duration_ms", kPropertyTypeInteger, 0, 0, MOTOR_MAX_RUN_TIME_MS),
            Property("degrees", kPropertyTypeInteger, MOTOR_TURN_DEFAULT_DEGREES, 1, 360),
            Property("distance_cm", kPropertyTypeInteger, 0, 0, 300),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            const std::string direction = properties["direction"].value<std::string>();
            const int speed = properties["speed"].value<int>();
            int duration_ms = properties["duration_ms"].value<int>();
            const int degrees = properties["degrees"].value<int>();
            int distance_cm = properties["distance_cm"].value<int>();
            if (distance_cm == 0 && duration_ms == 0 &&
                (direction == "forward" || direction == "backward")) {
                distance_cm = MOTOR_DIST_DEFAULT_CM;  // no distance or time given
            }
            const bool by_distance =
                distance_cm > 0 && (direction == "forward" || direction == "backward");
            const bool turning = direction == "left" || direction == "right";
            if (IsMotionLocked()) {
                return "{\"ok\":false,\"error\":\"battery_critical\",\"message\":\"Battery is "
                       "critically low; motors are locked. Tell the user Jarvis must be charged "
                       "now.\"}";
            }
            int applied_speed = 0;

            if (by_distance) {
                applied_speed = tuning_.dist_speed;
            } else if (direction == "forward" || direction == "backward") {
                applied_speed = speed * speed_factor_.load() / 100;
                if (speed != 0) {
                    applied_speed = std::max(applied_speed, 1);
                }
            } else if (turning) {
                applied_speed = tuning_.turn_speed;
                // With the IMU the turn is closed-loop and duration_ms is only a timeout.
                duration_ms = HasGyro() ? static_cast<int>(MOTOR_MAX_RUN_TIME_MS)
                                        : std::min(degrees * turn_ms_per_degree_.load(),
                                                   static_cast<int>(MOTOR_MAX_RUN_TIME_MS));
            } else {
                return "Invalid direction. Use forward, backward, left, or right.";
            }

            int left_speed = applied_speed;
            int right_speed = applied_speed;
            if (direction == "backward") {
                left_speed = -applied_speed;
                right_speed = -applied_speed;
            } else if (direction == "left") {
                left_speed = -applied_speed;
            } else if (direction == "right") {
                right_speed = -applied_speed;
            }

            if (by_distance) {
                duration_ms = static_cast<int>(std::min<uint32_t>(
                    DistanceMs(distance_cm, direction == "backward"), MOTOR_MAX_RUN_TIME_MS));
            }
            if (!Drive(left_speed, right_speed, static_cast<uint32_t>(duration_ms),
                       turning ? degrees : 0, by_distance ? distance_cm : 0)) {
                return "Motor command queue is unavailable.";
            }
            ESP_LOGI(TAG,
                     "Drive robot: direction=%s requested=%d%% factor=%d%% applied=%d%% "
                     "degrees=%d duration=%dms",
                     direction.c_str(), speed, speed_factor_.load(), applied_speed,
                     turning ? degrees : 0, duration_ms);
            std::string result = std::string("{\"ok\":true,\"direction\":\"") + direction +
                                 "\",\"applied_pwm_percent\":" + std::to_string(applied_speed);
            if (by_distance) {
                result += ",\"distance_cm\":" + std::to_string(distance_cm) +
                          ",\"distance_note\":\"timed at the calibrated speed\"";
            }
            if (turning) {
                result += ",\"degrees\":" + std::to_string(degrees) + ",\"turn_mode\":\"" +
                          (HasGyro() ? "gyro" : "timed") + "\"";
            } else {
                result += ",\"speed_factor_percent\":" + std::to_string(speed_factor_.load());
            }
            return result + ",\"duration_ms\":" + std::to_string(duration_ms) + "}";
        });

    mcp_server.AddTool(
        "self.robot.set_speed_factor",
        "System setting: set Jarvis's forward/backward speed factor in percent of full motor "
        "power (25-100). Use when the user says to go slower/faster or names a speed "
        "factor. Saved permanently. Turns are not affected.",
        PropertyList({
            Property("percent", kPropertyTypeInteger, MOTOR_SPEED_FACTOR_DEFAULT,
                     MOTOR_SPEED_FACTOR_MIN, MOTOR_SPEED_FACTOR_MAX),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            const int percent = properties["percent"].value<int>();
            speed_factor_.store(percent);
            SaveSetting(kSpeedFactorKey, percent);
            ESP_LOGI(TAG, "Speed factor set to %d%%", percent);
            return std::string("{\"ok\":true,\"speed_factor_percent\":") +
                   std::to_string(percent) + "}";
        });

    mcp_server.AddTool(
        "self.robot.calibrate_turn",
        "System setting: milliseconds of in-place turning per degree (1-40). If a 20 degree "
        "turn comes out too small, increase it; too large, decrease it. Saved permanently.",
        PropertyList({
            Property("ms_per_degree", kPropertyTypeInteger, MOTOR_TURN_MS_PER_DEGREE_DEFAULT,
                     1, MOTOR_TURN_MS_PER_DEGREE_MAX),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            const int ms_per_degree = properties["ms_per_degree"].value<int>();
            turn_ms_per_degree_.store(ms_per_degree);
            SaveSetting(kTurnMsPerDegreeKey, ms_per_degree);
            ESP_LOGI(TAG, "Turn calibration set to %d ms/deg", ms_per_degree);
            return std::string("{\"ok\":true,\"turn_ms_per_degree\":") +
                   std::to_string(ms_per_degree) + "}";
        });

    mcp_server.AddTool(
        "self.robot.run_motor",
        "Run exactly one of the robot's motors while keeping the other motor stopped. Always "
        "use this tool when the user says only the left motor/wheel or only the right "
        "motor/wheel. motor must be left or right. direction must be forward or backward. "
        "PWM = speed x the stored speed factor.",
        PropertyList({
            Property("motor", kPropertyTypeString, "left"),
            Property("direction", kPropertyTypeString, "forward"),
            Property("speed", kPropertyTypeInteger, 100, 0, 100),
            Property("duration_ms", kPropertyTypeInteger, 1000, 100, MOTOR_MAX_RUN_TIME_MS),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            const std::string motor = properties["motor"].value<std::string>();
            const std::string direction = properties["direction"].value<std::string>();
            const int speed = properties["speed"].value<int>();
            const int duration_ms = properties["duration_ms"].value<int>();

            if (IsMotionLocked()) {
                return "{\"ok\":false,\"error\":\"battery_critical\",\"message\":\"Battery is "
                       "critically low; motors are locked. Tell the user Jarvis must be charged "
                       "now.\"}";
            }
            if (motor != "left" && motor != "right") {
                return "Invalid motor. Use left or right.";
            }
            if (direction != "forward" && direction != "backward") {
                return "Invalid direction. Use forward or backward.";
            }

            int applied_speed = speed * speed_factor_.load() / 100;
            if (speed != 0) {
                applied_speed = std::max(applied_speed, 1);
            }
            const int signed_speed = direction == "forward" ? applied_speed : -applied_speed;
            const int left_speed = motor == "left" ? signed_speed : 0;
            const int right_speed = motor == "right" ? signed_speed : 0;
            if (!Drive(left_speed, right_speed, static_cast<uint32_t>(duration_ms))) {
                return "Motor command queue is unavailable.";
            }

            ESP_LOGI(TAG,
                     "Run one motor: motor=%s direction=%s requested=%d%% applied=%d%% "
                     "duration=%dms",
                     motor.c_str(), direction.c_str(), speed, applied_speed, duration_ms);
            return std::string("{\"ok\":true,\"motor\":\"") + motor +
                   "\",\"direction\":\"" + direction +
                   "\",\"requested_speed\":" + std::to_string(speed) +
                   ",\"applied_speed\":" + std::to_string(applied_speed) +
                   ",\"other_motor\":\"stopped\",\"duration_ms\":" +
                   std::to_string(duration_ms) + "}";
        });

    mcp_server.AddTool("self.robot.stop", "Stop the robot's motors immediately.", PropertyList(),
                       [this](const PropertyList&) -> ReturnValue {
                           Stop();
                           return true;
                       });

    mcp_server.AddTool("self.robot.get_status", "Get the robot's motor status.", PropertyList(),
                       [this](const PropertyList&) -> ReturnValue {
                           return std::string("{\"moving\":") + (IsMoving() ? "true" : "false") +
                                  ",\"left_speed\":" + std::to_string(LeftSpeed()) +
                                  ",\"right_speed\":" + std::to_string(RightSpeed()) +
                                  ",\"speed_factor_percent\":" +
                                  std::to_string(SpeedFactor()) + "}";
                       });
}
