# Attitude-Indicator-FPGA
## IMU-Based Attitude Indicator

This project implements a simple attitude indicator using an Adafruit BNO055 9-DOF IMU and a Go Board FPGA. The IMU provides accelerometer, gyroscope, and magnetometer data via I²C, which the FPGA retrieves and processes to estimate the device’s roll and pitch.

The FPGA performs basic filtering and sensor fusion to track orientation over time. The computed attitude is then mapped to one of nine discrete orientation states, each represented on the Go Board’s hex LED displays. The result is a compact, FPGA-based visualization of the IMU’s orientation relative to the horizon.
