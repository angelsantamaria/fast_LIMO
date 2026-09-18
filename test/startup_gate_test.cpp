#include "fast_limo/StartupGate.hpp"
#include <stdexcept>
void require(bool b) { if (!b) throw std::runtime_error("startup freshness regression"); }
int main() {
    fast_limo::StartupGate gate;
    require(!gate.ready(0));
    for (int i=0; i<=25; ++i) {
        double t = i * 0.1;
        gate.observe(0, 100+t, 100+t+0.01, t);
        gate.observe(1, 100+t, 100+t+0.01, t);
        require(gate.ready(t) == (i>=20));
    }
    gate.observe(0, 102.6, 102.8, 2.6); // Delayed scan resets the full settling period.
    require(!gate.ready(2.6));
    for (int i=27; i<=50; ++i) {
        double t=i*0.1;
        gate.observe(0, 100+t, 100+t+0.01, t);
        gate.observe(1, 100+t, 100+t+0.01, t);
    }
    require(gate.ready(5));
    require(!gate.ready(5.3)); // Missing stream, even if ROS time pauses.
    gate.observe(0, 106, 105.3, 5.3); // Future clock.
    require(!gate.ready(5.3));
    gate.observe(1, NAN, 105.3, 5.3);
    require(!gate.ready(5.3));
    fast_limo::StartupGate duplicate;
    duplicate.settle_time=0;
    duplicate.observe(0, 100, 100, 0);
    duplicate.observe(1, 100, 100, 0);
    require(duplicate.ready(0));
    duplicate.observe(1, 100, 100.01, .01);
    require(!duplicate.ready(.01));
}
