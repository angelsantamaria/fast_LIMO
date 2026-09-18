#pragma once
#include <Eigen/Geometry>
#include <cmath>

namespace fast_limo {
// Serialized by caller. Translation is checked against the map origin, never rebased.
class StationaryGate {
public:
    double duration = 5.0, translation = 0.15, rotation = 3.0 * M_PI / 180.0;
    bool passed = false, failed = false;
    void observe(double now, const Eigen::Vector3f& p, const Eigen::Quaternionf& q, bool healthy) {
        if (passed || failed) return;
        if (!p.allFinite() || !q.coeffs().allFinite() || q.norm() < 0.9 || q.norm() > 1.1 ||
            p.norm() > translation) { failed = true; return; }
        if (began_ < 0) {
            began_ = now;
            reference_ = q.normalized();
            const auto r = reference_.toRotationMatrix();
            if (std::abs(std::atan2(r(1,0), r(0,0))) > rotation) { failed = true; return; }
        }
        if (reference_.angularDistance(q.normalized()) > rotation) { failed = true; return; }
        if (!healthy) {
            // Allow map construction before the first correction sequence.
            if (stable_ >= 0 || now - began_ > 2.0) failed = true;
            return;
        }
        if (stable_ < 0) stable_ = now;
        passed = now - stable_ >= duration;
    }
private:
    double began_ = -1, stable_ = -1;
    Eigen::Quaternionf reference_ = Eigen::Quaternionf::Identity();
};
}
