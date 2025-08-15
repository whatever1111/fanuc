# Fanuc Driver Test Suite

This directory contains comprehensive tests for the Fanuc welding control frequency test tool.

## Test Structure

### 1. Algorithm Unit Tests
- **File**: `test_weld_algorithms.py`
- **Launch**: `test_weld_algorithms.test`
- **Purpose**: Tests mathematical algorithms without ROS dependencies
- **Coverage**: Waveform generation, pulse algorithms, numerical stability

### 2. ROS Integration Tests
- **File**: `test_weld_control_freq_ros.py`
- **Launch**: `test_weld_control_freq_ros.test`
- **Purpose**: Tests ROS message handling and node communication
- **Coverage**: Message publishing/subscribing, timing, latency measurement

## Running Tests

### Prerequisites
```bash
# Make sure you have built the workspace
cd ~/catkin_ws
catkin build fanuc_driver

# Source the workspace
source devel/setup.bash
```

### Run All Tests
```bash
# Run all tests with catkin
catkin test fanuc_driver

# Or run tests with rostest
rostest fanuc_driver test_weld_algorithms.test
rostest fanuc_driver test_weld_control_freq_ros.test
```

### Run Individual Test Suites

#### Algorithm Tests Only
```bash
# Using rostest
rostest fanuc_driver test_weld_algorithms.test

# Using rosrun (if executable)
rosrun fanuc_driver test_weld_algorithms.py
```

#### ROS Integration Tests Only
```bash
# Using rostest
rostest fanuc_driver test_weld_control_freq_ros.test

# Note: ROS integration tests require roscore to be running
```

### Run with Specific Parameters
```bash
# Run with custom parameters
rostest fanuc_driver test_weld_algorithms.test test_precision:=1e-8 test_iterations:=2000
```

## Test Output

### Expected Results
- All tests should pass with `PASS` status
- No `FAIL` or `ERROR` messages should appear
- Performance metrics will be logged

### Example Output
```
[ROSUNIT] test_sine_wave_generation ... ok
[ROSUNIT] test_pulse_probability_distribution ... ok
[ROSUNIT] test_signal_bounds_validation ... ok
...
Ran 8 tests in 2.345s
OK
```

## Test Coverage

### Algorithm Tests
- ✅ Sine wave generation accuracy
- ✅ Step wave generation accuracy  
- ✅ Pulse probability distribution
- ✅ Pulse duration control
- ✅ Signal bounds validation
- ✅ Numerical stability
- ✅ Complete algorithm integration

### ROS Integration Tests
- ✅ ROS message structure validation
- ✅ Message publishing/subscribing
- ✅ Waveform generation timing
- ✅ Pulse-enhanced signal in ROS context
- ✅ Latency measurement simulation

## Troubleshooting

### Common Issues

1. **"No module named 'fanuc_driver'"**
   ```bash
   # Make sure workspace is built and sourced
   catkin build fanuc_driver
   source devel/setup.bash
   ```

2. **"Unable to contact ROS master"**
   ```bash
   # Start roscore in another terminal
   roscore
   ```

3. **Test timeouts**
   ```bash
   # Increase timeout in .test files
   # <test time-limit="60.0" ...>
   ```

### Debug Mode
```bash
# Run with debug output
rostest --text fanuc_driver test_weld_algorithms.test

# Run with specific log level
ROS_LOG_LEVEL=DEBUG rostest fanuc_driver test_weld_control_freq_ros.test
```

## Contributing

When adding new tests:

1. Add test functions to appropriate test files
2. Update .test launch files if needed
3. Add new test files to CMakeLists.txt
4. Update this README with new test descriptions

## Test Data

Test parameters are configured in the .test launch files and can be overridden:

- `test_precision`: Numerical precision for algorithm tests (default: 1e-6)
- `test_iterations`: Number of iterations for statistical tests (default: 1000)
- `test_duration`: Duration for timing tests (default: 2.0s)
- `sample_rate`: Sample rate for signal generation tests (default: 20Hz)
- `threshold`: Threshold for latency measurement tests (default: 2.0)