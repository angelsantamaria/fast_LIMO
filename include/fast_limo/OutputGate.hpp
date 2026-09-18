#pragma once

#include <cmath>
#include <mutex>

namespace fast_limo {
// Thread-safe health shared by the LiDAR, IMU and watchdog callbacks.
class OutputGate {
public:
    void record(bool corrected, double stamp, double steady_now) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!corrected || !std::isfinite(stamp) || stamp <= stamp_) {
            consecutive_ = 0;
            return;
        }
        if (steady_now - received_ > timeout || stamp - stamp_ > timeout) consecutive_ = 0;
        stamp_ = stamp;
        received_ = steady_now;
        if (consecutive_ < 3) ++consecutive_;
    }

    bool ready(bool calibrated, double now, double steady_now) {
        std::lock_guard<std::mutex> lock(mutex_);
        const double age = now - stamp_;
        if (!calibrated || !std::isfinite(now) || age < -0.05 || age > timeout ||
            steady_now - received_ > timeout) {
            consecutive_ = 0;
            return false;
        }
        return consecutive_ >= 3;
    }

    static constexpr double timeout = 0.5;

private:
    std::mutex mutex_;
    double stamp_ = 0.0;
    double received_ = 0.0;
    int consecutive_ = 0;
};
}  // namespace fast_limo
