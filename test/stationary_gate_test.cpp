#include "fast_limo/StationaryGate.hpp"
#include <stdexcept>
using namespace fast_limo;
void check(bool v) { if (!v) throw std::runtime_error("stationary gate regression"); }
int main() {
    Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    Eigen::Quaternionf identity = Eigen::Quaternionf::Identity();
    StationaryGate good;
    good.observe(0, zero, identity, false);
    good.observe(1, zero, identity, true);
    good.observe(5.9, zero, identity, true); check(!good.passed);
    good.observe(6, zero, identity, true); check(good.passed);
    StationaryGate huge;
    huge.observe(0, Eigen::Vector3f(20,0,0), identity, true); check(huge.failed);
    StationaryGate drift;
    drift.observe(0, zero, identity, true);
    drift.observe(1, Eigen::Vector3f(.16,0,0), identity, true); check(drift.failed);
    StationaryGate turn;
    turn.observe(0, zero, identity, true);
    turn.observe(1, zero, Eigen::Quaternionf(Eigen::AngleAxisf(.1, Eigen::Vector3f::UnitZ())), true);
    check(turn.failed);
    StationaryGate tilted;
    tilted.observe(0, zero, Eigen::Quaternionf(Eigen::AngleAxisf(.3, Eigen::Vector3f::UnitX())), true);
    check(!tilted.failed);
    StationaryGate lost;
    lost.observe(0, zero, identity, true); lost.observe(1, zero, identity, false); check(lost.failed);
    StationaryGate missing;
    missing.observe(0, zero, identity, false); missing.observe(2.1, zero, identity, false); check(missing.failed);
    StationaryGate invalid;
    invalid.observe(0, Eigen::Vector3f(NAN,0,0), identity, true); check(invalid.failed);
}
