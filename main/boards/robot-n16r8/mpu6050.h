#ifndef _ROBOT_N16R8_MPU6050_H_
#define _ROBOT_N16R8_MPU6050_H_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>

#include <driver/i2c_master.h>

// MPU-6050 IMU service: the only code that talks to the chip.
//
// The chip samples accel + gyro on its own clock at exactly 100 Hz into its FIFO
// (12-byte frames: accel X/Y/Z, gyro X/Y/Z). One task drains the FIFO every 30 ms, so every
// sample is used with an exact dt of 10 ms whatever the CPU load is, and the shared I2C
// bus (camera SCCB, INA219) only sees one short burst per 30 ms. From that stream it keeps:
//   - the heading (yaw = gyro projected on "up", left/counter-clockwise positive),
//   - the latest accel/gyro for the body sense and the console,
//   - an odometer for straight drives (accelerometer), armed by the motor controller.
// At startup it calibrates gyro bias and "up" while the robot stands still and repeats the
// calibration if the robot was moved; afterwards the gyro bias keeps being refined
// whenever the robot is still.
class Mpu6050 {
public:
    Mpu6050(i2c_master_bus_handle_t bus, uint8_t address);

    bool IsPresent() const { return device_ != nullptr; }
    // True once the startup calibration is done and samples are flowing.
    bool HeadingOk() const { return calibrated_.load(); }

    // Total yaw since startup in degrees (left/counter-clockwise positive).
    float Heading() const { return heading_.load(); }
    // Latest values from the sample stream (no bus access).
    bool ReadYawRate(float& dps);
    bool ReadMotion(float accel_g[3], float gyro_dps[3]);  // accel g, bias-free gyro deg/s
    bool ReadAccelXY(float& x_g, float& y_g);
    uint32_t SampleCount() const { return sample_count_.load(); }

    // "Up" unit vector (chip axes) and |accel| at rest, both from the calibration.
    const float* Up() const { return up_; }
    float RestG() const { return rest_g_; }
    float MountTiltDegrees() const;

    float YawScale() const { return yaw_scale_; }
    void SetYawScale(float scale);
    void SaveYawScale() const;  // NVS: call from a task with an internal-RAM stack

    // Odometer along one accel axis (axis 0 = X, 1 = Y; sign +-1 = which way is forward).
    // It first measures the accel bias over 20 samples (robot standing), then integrates
    // speed (only in the travel direction: travel +1 forward, -1 backward) and distance.
    void StartOdometer(int axis, int sign, float travel);
    void StopOdometer() { odo_state_.store(kOdoOff); }
    bool OdometerReady() const { return odo_state_.load() == kOdoRunning; }
    float OdometerMeters() const { return odo_distance_.load(); }
    float OdometerSpeed() const { return odo_speed_.load(); }

    int FifoResets() const { return fifo_resets_.load(); }

private:
    enum OdoState { kOdoOff, kOdoBias, kOdoRunning };

    i2c_master_dev_handle_t device_ = nullptr;
    float bias_[3] = {0, 0, 0};
    float up_[3] = {0, 0, 1};
    float rest_g_ = 1.0f;
    float yaw_scale_ = 1.0f;
    int still_samples_ = 0;

    std::atomic<bool> calibrated_{false};
    std::atomic<float> heading_{0.0f};
    std::atomic<uint32_t> sample_count_{0};
    std::atomic<int> fifo_resets_{0};
    mutable std::mutex latest_mutex_;
    float latest_accel_[3] = {0, 0, 1};
    float latest_gyro_[3] = {0, 0, 0};
    float latest_yaw_rate_ = 0;

    std::atomic<int> odo_state_{kOdoOff};
    int odo_axis_ = 0;
    float odo_sign_ = 1;
    float odo_travel_ = 1;
    float odo_bias_sum_ = 0;
    int odo_bias_count_ = 0;
    float odo_bias_ = 0;
    std::atomic<float> odo_speed_{0.0f};
    std::atomic<float> odo_distance_{0.0f};

    bool Configure();
    void ResetFifo();
    int ReadFrames(float accel[][3], float gyro[][3], int max_frames);
    void Run();
    bool Calibrate(int attempt);
    void Process(const float accel[3], const float gyro_raw[3]);
    bool ReadRegister(uint8_t reg, uint8_t* buffer, size_t length);
    bool WriteRegister(uint8_t reg, uint8_t value);
};

#endif  // _ROBOT_N16R8_MPU6050_H_
