// INA219 battery monitor (1S Li-ion, 5600 mAh) and protection. One sample per second;
// decisions use readings taken with the motors stopped (no inrush sag):
// < 3.5 V low (warning), < 3.3 V critical (motors locked), unlocked again above 3.6 V.
#pragma once

namespace power {
struct Reading {
    bool ok = false;
    int mv = 0;
    int ma = 0;
    int percent = -1;   // from the resting voltage
    int level = 0;      // 0 good, 1 < 35 %, 2 < 20 %, 3 < 10 % (face eye color)
    bool low = false;
    bool critical = false;
};
void Start();
Reading Get();
}  // namespace power
