#ifndef _ROBOT_N16R8_BODY_SENSE_H_
#define _ROBOT_N16R8_BODY_SENSE_H_

#include <atomic>
#include <cstdint>
#include <string>

class Mpu6050;
class MotorController;
class Display;

// Body awareness from the IMU (like NeuroVision-Gemini nv/senses.py): being lifted,
// tipping over and bumping into something. On an event the motors stop, the face looks
// surprised and, when Jarvis is idle, the event is handed to the AI as a short message so
// it decides how to respond. The latest state is reported through StatusJson().
class BodySense {
public:
    BodySense(Mpu6050* imu, MotorController* motor, Display* display);

    std::string StatusJson() const;

private:
    enum Event { kNone, kLifted, kPutDown, kTipped, kUpright, kBump };

    Mpu6050* imu_;
    MotorController* motor_;
    Display* display_;
    std::atomic<int> tilt_x10_{0};
    std::atomic<bool> held_{false};
    std::atomic<bool> tipped_{false};
    std::atomic<int> last_event_{kNone};
    std::atomic<int64_t> last_event_us_{0};
    int64_t last_ai_event_us_ = 0;

    void Loop();
    void Report(Event event, const char* detail);
};

#endif  // _ROBOT_N16R8_BODY_SENSE_H_
