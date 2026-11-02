#ifndef _ROBOT_N16R8_BATTERY_GAUGE_H_
#define _ROBOT_N16R8_BATTERY_GAUGE_H_

#include <atomic>
#include <mutex>
#include <string>

#include "ina219.h"

// Battery state of charge, ported from NeuroVision-Gemini (nv/power.py).
// The percent is tracked by counting the current drawn from the capacity, because the
// voltage sags whenever the motors pull. It starts from the resting voltage (or from the
// value saved in NVS if the cell was not charged meanwhile). While idle, the count slowly
// blends toward the voltage curve, and at the full voltage it snaps to 100%.
class BatteryGauge {
public:
    // level: 0 good, 1 getting low (<35%), 2 low (<20%), 3 very low (<10%).
    struct Snapshot {
        bool ok = false;
        int percent = 0;
        int level = 0;
        float volts = 0.0f;
        float amps = 0.0f;
        float watts = 0.0f;
        float avg_amps = 0.0f;
        float peak_amps = 0.0f;
    };

    explicit BatteryGauge(Ina219* sensor);

    Snapshot Get() const;
    std::string StatusJson() const;

private:
    Ina219* sensor_;
    mutable std::mutex mutex_;
    Snapshot snapshot_;
    float charge_ = -1.0f;  // remaining percent from the current count; <0 = not started
    int64_t read_at_us_ = 0;
    int64_t saved_at_us_ = 0;
    int fails_ = 0;
    int saved_charge_x100_ = -1;  // read from NVS at construction (main task)
    int saved_mv_ = -1;

    void Loop();
    void Count(float volts, float amps, int64_t now_us);
    float PercentOfVolts(float volts) const;
    void UpdateLevel(int percent);
    float LoadCharge(float volts, float volts_percent, bool idle);
    void SaveCharge(float volts);
};

#endif  // _ROBOT_N16R8_BATTERY_GAUGE_H_
