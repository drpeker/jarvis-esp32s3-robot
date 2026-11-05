// MPU-6050: the only code that talks to the chip. One task reads the measurement registers
// (accel, temperature, gyro: 14 bytes in one burst, always aligned) every 10 ms and
// integrates with the measured dt. The chip FIFO is not used: a FIFO that slips by one word
// still looks like 1 g and silently put "up" on the wrong axis.
// Keeps the heading (yaw about "up", left / counter-clockwise positive), the tilt, and
// refines the gyro bias whenever the robot is still.
#pragma once

namespace imu {
bool Start();            // false: no IMU (turns fall back to timing)
bool Ready();            // calibrated and running
float Heading();         // degrees since boot, left positive
float YawRate();         // deg/s
float TiltDegrees();     // current tilt from the boot "up"
int ReadErrors();
void Debug(float up[3], float accel[3]);  // console
float YawScale();
void SetYawScale(float scale);  // also saved in NVS (robot/gyro_scale)
}  // namespace imu
