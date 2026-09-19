# Sensor fusion test harness

A planar differential drive robot fuses a 1 kHz IMU with 50 Hz wheel odometry
in a six-state EKF (`x, y, theta, v`, accel bias, gyro bias). The harness runs a
30 s scenario with a good and a cheap IMU, prints position, speed and heading
error for the fused estimate against IMU-only and wheel-only dead reckoning,
then times the filter.

## Build

CMake 3.20+ and a C++20 compiler. GoogleTest is fetched if it isn't installed.

```
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release
build/Release/fusion_harness      # build/fusion_harness on Linux/macOS
```

## Assumptions

Motion is planar, the initial pose is known, and the wheels have no slip or
scale error. The filter is given the true sensor noise parameters. Only speed
and the two IMU biases are observable, so position and heading drift without
an external reference.
