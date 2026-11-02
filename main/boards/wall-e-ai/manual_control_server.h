#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <string>

class ManualControlServer {
public:
    using PwmWriter = std::function<void(unsigned char channel, unsigned short duty)>;
    using ExitCallback = std::function<void()>;

    ManualControlServer(PwmWriter write_pwm, std::mutex* camera_mutex, ExitCallback exit_callback);
    void Initialize();
    bool Start();
    void Stop();
    bool DriveFor(const std::string& direction, int duration_ms);
    bool SetServoAngle(int index, int angle);
    bool ApplyNamedPose(const std::string& pose);
    void SetAiSpeaking(bool speaking);
    void SetAiEmotion(const std::string& emotion);
    void PlayBootAnimation();
    void RequestExit();
    bool active() const { return active_.load(); }

private:
    PwmWriter write_pwm_;
    std::mutex* camera_mutex_ = nullptr;
    ExitCallback exit_callback_;
    std::atomic_bool initialized_{false};
    std::atomic_bool active_{false};
};
