#ifndef _MOCHAN_V2_MOTOR_CONTROLLER_H_
#define _MOCHAN_V2_MOTOR_CONTROLLER_H_

#include <atomic>
#include <cstdint>

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "mpu6050.h"

class MotorController {
public:
    // Per-motor calibration (motor 0 = left, 1 = right; dir 0 = IN1/forward, 1 = reverse).
    // Logical speed 1..100 maps to min_duty..100% PWM, scaled by the motor's gain.
    struct Tuning {
        int min_duty[2][2];
        int gain[2];
        int turn_speed;
        int turn_lead_x10;  // gyro turn stops this many tenths of a degree early
        int kick_ms;
        int brake_ms;
        int pwm_hz;
        int accel_axis;  // IMU axis pointing forward: 0 = X, 1 = Y
        int accel_sign;  // +1 or -1
    };

    MotorController(gpio_num_t left_in1, gpio_num_t left_in2, gpio_num_t right_in1,
                    gpio_num_t right_in2);
    ~MotorController();

    // target_degrees > 0 with a gyro attached: run until the gyro has measured that
    // much rotation (duration_ms is then only the safety timeout).
    bool Drive(int left_speed, int right_speed, uint32_t duration_ms, int target_degrees = 0,
               int target_cm = 0);
    void Stop();
    bool IsMoving() const;
    int LeftSpeed() const;
    int RightSpeed() const;
    int SpeedFactor() const;
    int TurnMsPerDegree() const;
    // Battery protection: while locked every motion command is refused (stop still works).
    void SetMotionLocked(bool locked);
    bool IsMotionLocked() const;
    void SetGyro(Mpu6050* gyro) { gyro_ = gyro; }
    Tuning GetTuning() const;
    void SetTuning(const Tuning& tuning, bool save);
    // Calibration: while raw mode is on, queued commands are refused and SetRawDuty drives
    // the H-bridge directly (signed percent of full PWM, no mapping/kick/gain).
    void SetRawMode(bool on);
    void SetRawDuty(int left_percent, int right_percent);
    // Result of the last gyro turn (final measured angle, tenths of a degree) and a
    // counter that increases when a gyro turn finishes.
    int LastTurnFinalX10() const { return last_turn_final_x10_.load(); }
    uint32_t TurnCount() const { return turn_count_.load(); }
    // Result of the last accelerometer distance drive (estimated millimetres).
    int LastDistanceMm() const { return last_distance_mm_.load(); }
    uint32_t DistanceCount() const { return distance_count_.load(); }
    bool HasGyro() const { return gyro_ != nullptr && gyro_->HeadingOk(); }

private:
    struct Command {
        int left_speed;
        int right_speed;
        uint32_t duration_ms;
        int target_degrees;
        int target_cm;
    };

    gpio_num_t pins_[4];
    const ledc_channel_t channels_[4] = {
        LEDC_CHANNEL_1,
        LEDC_CHANNEL_2,
        LEDC_CHANNEL_3,
        LEDC_CHANNEL_4,
    };
    QueueHandle_t command_queue_ = nullptr;
    TaskHandle_t task_handle_ = nullptr;
    std::atomic<bool> moving_{false};
    std::atomic<int> left_speed_{0};
    std::atomic<int> right_speed_{0};
    int last_physical_left_speed_ = 0;
    int last_physical_right_speed_ = 0;
    std::atomic<int> speed_factor_{0};
    std::atomic<int> turn_ms_per_degree_{0};
    std::atomic<bool> motion_locked_{false};
    std::atomic<bool> raw_mode_{false};
    std::atomic<int> last_turn_final_x10_{0};
    std::atomic<uint32_t> turn_count_{0};
    std::atomic<int> last_distance_mm_{0};
    std::atomic<uint32_t> distance_count_{0};
    Tuning tuning_;
    Mpu6050* gyro_ = nullptr;
    float coast_lead_ = 7.0f;  // learned from the turns (degrees after the stop point)

    static void TaskEntry(void* arg);
    void TaskLoop();
    void ConfigurePwm();
    void ApplySpeeds(int left_speed, int right_speed);
    bool RunGyroTurn(Command turn, Command& next);
    void SetSpeedsNow(int left_speed, int right_speed);
    bool StopRequested(Command& next, TickType_t wait);
    bool RunDistanceDrive(Command drive, Command& next);
    void Brake();
    void SetMotor(int speed, int motor, bool raw = false);
    void SetDuty(ledc_channel_t channel, uint32_t duty);
    void LoadSettings();
    void SaveSetting(const char* key, int value);
    void RegisterMcpTools();
};

#endif  // _MOCHAN_V2_MOTOR_CONTROLLER_H_
