// MX1508 tracks, tuned from the NVS keys of namespace "robot"
// (kept on the robot across firmware updates):
//  - turns: in place at turn_spd, closed loop on the MPU-6050 heading; stops `lead` degrees
//    early (learned from the coast of previous turns), brakes, then corrects to +-3 deg;
//  - forward / backward: timed, at dist_spd, 20 cm = ms20_fwd / ms20_back ms;
//  - logical speed 1..100 maps to min_duty..100 % (min duty 80 %, never lower).
// Moves run in the caller's task (blocking); Stop() from any task ends them at once.
#pragma once

namespace motors {
struct Tuning {
    int min_duty[2][2];  // [left/right][forward/reverse] percent
    int gain[2];
    int turn_speed, lead_x10, kick_ms, brake_ms, pwm_hz, dist_speed, ms20[2];
};
struct Result {
    bool ok;
    float value;       // turn: degrees turned; drive: ms driven
    const char* note;  // why it ended early, or ""
};

void Start();
Result Turn(bool left, int degrees);
Result Distance(bool forward, int cm);
Result Drive(bool forward, int ms, int speed_percent);
void Stop();
bool Moving();
void SetLocked(bool locked);  // battery critical: everything except Stop is refused
bool Locked();
Tuning GetTuning();
void SetTuning(const Tuning& t, bool save);
float TurnLead();
}  // namespace motors
