#include "body_sense.h"

#include <cmath>
#include <cstdio>

#include "motor_controller.h"
#include "mpu6050.h"

namespace {
constexpr float kTipDeg = 35.0f;       // tipped over above this tilt
// Moved by hand (motors off): |a| away from the rest value or the body turning.
constexpr float kLiftAccelG = 0.12f;
constexpr float kLiftGyroDps = 25.0f;
constexpr float kLiftTiltDeg = 10.0f;

float Length(const float v[3]) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

float Dot(const float a[3], const float b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
}  // namespace

BodySense::BodySense(Mpu6050* imu, MotorController* motor) : imu_(imu), motor_(motor) {}

std::string BodySense::StatusJson() const {
    float accel[3], gyro[3];
    if (!imu_->ReadMotion(accel, gyro)) {
        return "{\"imu\":\"not_ready\"}";
    }
    const float* up = imu_->Up();
    const float len = Length(accel);
    const float tilt =
        len > 0.2f ? std::acos(std::fmax(-1.0f, std::fmin(1.0f, Dot(accel, up) / len))) *
                         180.0f / static_cast<float>(M_PI)
                   : 90.0f;
    const float accel_off = std::fabs(len - imu_->RestG());
    const float spin = Length(gyro);
    const bool moving = motor_ != nullptr && motor_->IsMoving();
    const bool held = !moving && (accel_off > kLiftAccelG || spin > kLiftGyroDps ||
                                  (tilt > kLiftTiltDeg && tilt <= kTipDeg));

    char buffer[200];
    snprintf(buffer, sizeof(buffer),
             "{\"tilt_deg\":%.1f,\"lifted_or_held\":%s,\"tipped_over\":%s,"
             "\"rotation_dps\":%.0f,\"imu_mount_tilt_deg\":%.1f}",
             tilt, held ? "true" : "false", tilt > kTipDeg ? "true" : "false", spin,
             imu_->MountTiltDegrees());
    return buffer;
}
