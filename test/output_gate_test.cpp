#include "fast_limo/OutputGate.hpp"
#include <stdexcept>

void require(bool value) {
    if (!value) throw std::runtime_error("Output gate regression");
}

int main() {
    fast_limo::OutputGate gate;
    require(!gate.ready(false, 100, 10));
    require(!gate.ready(true, 100, 10));  // IMU alone never opens the gate.
    gate.record(true, 100, 10);
    require(!gate.ready(true, 100.01, 10.01));
    gate.record(true, 100.1, 10.1);
    require(!gate.ready(true, 100.11, 10.11));
    gate.record(true, 100.2, 10.2);
    require(gate.ready(true, 100.21, 10.21));
    gate.record(false, 100.3, 10.3);
    require(!gate.ready(true, 100.31, 10.31));
    for (int i = 4; i <= 6; ++i) gate.record(true, 100 + i*.1, 10 + i*.1);
    require(gate.ready(true, 100.61, 10.61));
    require(!gate.ready(true, 101.2, 11.2)); // LiDAR disappears.
    gate.record(true, 101.3, 11.3);
    require(!gate.ready(true, 101.31, 11.31)); // Needs a full recovery sequence.
    gate.record(true, 101.4, 11.4);
    gate.record(true, 101.5, 11.5);
    require(gate.ready(true, 101.51, 11.51));
    require(!gate.ready(true, 100, 11.52)); // Future sensor timestamps / clock rewind.
    gate.record(true, 101.6, 11.6);
    gate.record(true, 101.7, 11.7);
    gate.record(true, 101.8, 11.8);
    require(!gate.ready(true, 101.81, 12.5)); // Paused ROS time cannot keep health alive.
    return 0;
}
