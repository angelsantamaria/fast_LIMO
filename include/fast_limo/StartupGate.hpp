#pragma once
#include <cmath>
#include <limits>

namespace fast_limo {
// Caller serializes access. Steady time measures continuity; ROS time measures age.
class StartupGate {
public:
    double max_age = 0.1, settle_time = 2.0, max_gap = 0.25;
    double ages[2] = {INFINITY, INFINITY};
    void observe(int sensor, double stamp, double now, double steady) {
        bool gap = false;
        for (int i = 0; i < 2; ++i)
            gap = gap || (seen_[i] && steady - received_[i] > max_gap);
        ages[sensor] = now - stamp;
        const bool valid = std::isfinite(stamp) && stamp > 0 &&
            std::isfinite(ages[sensor]) && ages[sensor] >= -0.05 && ages[sensor] <= max_age &&
            (!seen_[sensor] || stamp > stamps_[sensor]);
        if (gap || !valid) since_ = -1;
        seen_[sensor] = true;
        good_[sensor] = valid;
        stamps_[sensor] = stamp;
        received_[sensor] = steady;
        if (good_[0] && good_[1] && !gap && since_ < 0) since_ = steady;
    }
    bool ready(double steady) {
        for (int i = 0; i < 2; ++i)
            if (!seen_[i] || !good_[i] || steady - received_[i] > max_gap) {
                since_ = -1;
                return false;
            }
        return since_ >= 0 && steady - since_ >= settle_time;
    }
private:
    bool seen_[2] = {false, false}, good_[2] = {false, false};
    double stamps_[2] = {}, received_[2] = {}, since_ = -1;
};
}
