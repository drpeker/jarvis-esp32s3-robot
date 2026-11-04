#ifndef _ROBOT_N16R8_BODY_SENSE_H_
#define _ROBOT_N16R8_BODY_SENSE_H_

#include <string>

class Mpu6050;
class MotorController;

// Body awareness from the IMU, read only when asked (self_awareness tool). There is no
// background task: Jarvis does not watch its posture on its own, stop the motors or
// start a conversation turn by itself; it just reports the current tilt and motion.
class BodySense {
public:
    BodySense(Mpu6050* imu, MotorController* motor);

    std::string StatusJson() const;

private:
    Mpu6050* imu_;
    MotorController* motor_;
};

#endif  // _ROBOT_N16R8_BODY_SENSE_H_
