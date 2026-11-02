#ifndef _ROBOT_N16R8_ROBOT_CONSOLE_H_
#define _ROBOT_N16R8_ROBOT_CONSOLE_H_

class MotorController;
class Mpu6050;
class BatteryGauge;

// Console commands for motor tests and calibration (see net_console.h).
void RegisterRobotConsole(MotorController* motor, Mpu6050* imu, BatteryGauge* gauge);

#endif  // _ROBOT_N16R8_ROBOT_CONSOLE_H_
