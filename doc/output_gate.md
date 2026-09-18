# Localization output gate

Odometry, local TF and processed/debug point clouds are withheld until IMU
calibration completes and three consecutive LiDAR updates each finish with at
least six matched points and a finite estimated pose/velocity. Building the
initial map is allowed internally; seeding an empty map does not open the gate.

An unsuccessful scan closes the gate. Successful scan timestamps and their
steady-clock reception age must remain within 0.5 seconds; recovery requires
three consecutive successful updates again. Stale, repeated, backwards or
nonfinite IMU input is rejected. Scan waits for IMU coverage time out after
200 ms instead of blocking indefinitely.

`fast_limo/localization_healthy` (`std_msgs/Bool`, 20 Hz) is the health heartbeat,
including `false` during initialization. With namespace `b2` it is published on
`/b2/fast_limo/localization_healthy`. Consumers must check both its value and
freshness. The real stationary GNSS calibration requires this heartbeat as well
as local TF; missing/false/stale health resets its calibration window. After
global calibration, loss of health suppresses global position publication and
makes readiness false, while the frozen geodetic datum remains unchanged.

Odometry/TF retain IMU acquisition timestamps; processed clouds retain scan
timestamps. Invalid localization is not restamped as current. Existing TF in a
consumer's buffer cannot be retracted when the gate closes; consumers must use
freshness checks and the health heartbeat, not just the existence of a TF edge.

This gate prevents publishing uncorrected IMU-only localization. It does not
repair sensor time synchronization, mounting calibration, IMU units or poor
LiDAR geometry, nor does a match-count threshold guarantee absolute accuracy.
Investigate the logged synchronization/matching failures if the gate stays closed.
