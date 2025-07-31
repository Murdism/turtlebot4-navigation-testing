## 🎯 Project Overview

This project is an advanced automated testing solution for autonomous robot navigation that validates TurtleBot4's ability to navigate from point A to point B using the Nav2 stack. The system captures comprehensive performance metrics through statistical analysis of repeated test runs, making it ideal for rigorous validation, regression testing, and performance benchmarking of navigation algorithms.

### Key Features

- **Containerized ROS 2 Environment**: Complete Docker setup with ROS 2 Jazzy Jellyfish
- **Statistical Analysis via Repetitions**: Critical for reliable performance validation through multiple test runs
- **Automated Navigation Testing**: Send navigation goals and validate robot performance  
- **Comprehensive Performance Metrics**: Duration, success rate, path efficiency, deviation analysis, and recovery behaviors
- **Path Trajectory Analysis**: Compare planned vs actual robot paths for navigation quality assessment
- **Multiple Test Modes**: Single tests, batch testing, and repeated runs for statistical confidence
- **Flexible Configuration**: Support for multiple worlds and test scenarios

## 🏗️ Architecture

### Core Components

1. **Launch File** (`nav2_test_suite.launch.py`) - Orchestrates the entire test environment
2. **Nav2 Test Node** (`Nav2TestNode`) - Core navigation testing engine with path tracking
3. **AMCL Pose Initializer** (`amcl_pose_initializer`) - Reliable robot pose initialization

### Test Flow

```
1. Launch TurtleBot4 Gazebo Simulation + Nav2 Stack
2. Teleport Robot to Start Position  
3. Initialize AMCL Pose
4. Send Navigation Goal
5. Monitor Navigation Progress & Collect Metrics
6. Generate Detailed Test Reports
7. Repeat for Multiple Runs (Optional)
```

## 🚀 Quick Start

### Prerequisites

- **Docker** (with NVIDIA Container Toolkit for GPU support)
- **Git**
- At least 8GB RAM and 4 CPU cores recommended

### 1. Clone Repository

```bash
git clone https://github.com/Murdism/turtlebot4-navigation-testing.git
cd turtlebot4-navigation-testing
```

### 2. Build Docker Environment

```bash
# Make scripts executable
chmod +x build_docker.sh run_docker.sh

# Build the Docker image (includes ROS 2 Jazzy + TurtleBot4)
./build_docker.sh
```

### 3. Run the Container

```bash
# Start the container with GUI support
./run_docker.sh
```

### 4. Run Your First Test

Inside the container:
```
colcon build
source install/setup.bash
cd /home/tester/turtlebot4-navigation-testing/
```

```bash
# Basic navigation test (depot world: 0,0 → 3,2)
ros2 launch nav2_performance_tests nav2_test_suite.launch.py \
    start:="0.0,0.0" goal:="3.0,2.0"

# Check results
ls reports/
cat reports/test_01_summary.yaml
```

## 🧪 Testing Capabilities

### Available Test Worlds

- **depot** - Simple warehouse environment (default)
- **warehouse** - Complex multi-room facility
- **maze** - Challenging maze navigation

### 🔺 Available Navigation Options

You can run:

Manual Teleop Tests: Manually control the robot using the keyboard or Gazebo GUI sliders

Single Run Test: Start the robot from a fixed location and navigate to a goal once

Repetitive Runs: Repeat the same test multiple times to analyze consistency, timing, and deviations

Batch Tests: Run a YAML-defined suite of different start-goal navigation tests

Sequential Live Runs: After a test completes, run another by specifying new start_x, goal_x, etc. without restarting Gazebo


A complete ROS 2-based test automation framework to validate and evaluate TurtleBot4 navigation performance in Gazebo. Built for the Test Field Engineer Challenge, this solution supports manual and automated navigation testing, real-time metrics collection, and detailed YAML reports.

✅ Designed to: Set up simulation, manually test robot motion, automate navigation A → B, and generate performance reports.

```
ℹ️ Launch File Input Options:

All inputs below can be set via launch file or passed through ros2 launch:

start: "x,y" or "x,y,z,yaw" (e.g., "1.0,2.0" or "1.0,2.0,0.0,1.57")

goal: "x,y" or "x,y,z,yaw"

start_x, start_y, start_z, start_yaw: Individual floats

goal_x, goal_y, goal_z, goal_yaw: Individual floats

dist_thres: Distance threshold (meters), default 0.30

test_name: Custom name for test run

repetitions: How many times to repeat the test

world_name: World to launch (depot, warehouse, maze)

entity_name: Robot name in Gazebo (default turtlebot4)

test_file: Path to batch test YAML file
```


```
ℹ️ Nav2TestNode Input Options

All parameters below can be passed
with:
ros2 run nav2_performance_tests Nav2TestNode using --ros-args -p:

start: "x,y" or "x,y,z,yaw" (e.g., "1.0,2.0" or "1.0,2.0,0.0,1.57")

goal: "x,y" or "x,y,z,yaw"

start_x, start_y, start_z, start_yaw: Start position (individual float values)

goal_x, goal_y, goal_z, goal_yaw: Goal position (individual float values)

dist_thres: Distance threshold in meters

test_name: Name for the test run (e.g., test_repeat01)

repetitions: Number of repetitions (default 1)

world_name: World name for simulation context (used in report and logging)

test_file: Optional path to a batch YAML file (used if running batch mode)

entity_name: Name of the TurtleBot4 robot in Gazebo

use_sim_time: Set to True when using simulation clock (default True)
```
### Single Navigation Test

```bash
# Simple point-to-point navigation
ros2 launch nav2_performance_tests nav2_test_suite.launch.py \
    start:="0.0,0.0" goal:="4.0,3.0" \
    test_name:=basic_nav_test
```
To run another test you do not need to restart:
First open a new instance or using tmux a new termianl inside the container
To open another docker termainl:
```bash
docker exec -it ros2_nav_container bash
# inside the container run the following
source install/setup.bash
cd /home/tester/turtlebot4-navigation-testing/
```

use Nav2TestNode to send new start and goal points
```
bash
ros2 run nav2_performance_tests Nav2TestNode.py --ros-args -p
start:  "1.0,2.0" -p goal: "4.0,3.0" 
```
### Multi-Run Performance Testing

```bash
# Run 10 iterations for statistical analysis
ros2 launch nav2_performance_tests nav2_test_suite.launch.py \
    start:="0.5,0.5" goal:="4.5,3.5" \
    repetitions:=10 \
    test_name:=performance_analysis \
    dist_thres:=0.15
```

### Different World Environments

```bash
# Test in warehouse environment
ros2 launch nav2_performance_tests nav2_test_suite.launch.py \
    world_name:=warehouse \
    start:="1.0,1.0,0.0,0.0" goal:="8.0,5.0,0.0,1.57" \
    test_name:=warehouse_test
```

### Running Additional Tests (While Simulation Running)

```bash
# Run another test without restarting simulation
ros2 run nav2_performance_tests Nav2TestNode \
    --ros-args -p start_x:=2.0 -p goal_x:=6.0 -p test_name:="test2"
```

## 📊 Performance Metrics & Data Collection Strategy

### Why We Track Planned vs Actual Paths

**Planned Path (`/plan` topic)**:
- **Source**: Nav2 global planner output showing the intended navigation route
- **Purpose**: Represents optimal path computation considering obstacles and map constraints
- **Analysis Value**: Reveals planning algorithm quality, path smoothness, and obstacle avoidance strategy

**Actual Path (`/amcl_pose` topic)**:  
- **Source**: AMCL localization estimates showing where the robot believes it traveled
- **Purpose**: Captures real-world navigation performance including localization uncertainty
- **Analysis Value**: Reflects actual system behavior, control accuracy, and localization quality

This dual-path approach enables detection of both **planning issues** (suboptimal routes) and **execution issues** (poor path following), making it the industry standard for Nav2 performance evaluation.

### Critical Role of Test Repetitions

**Statistical Confidence**: Single test runs can be misleading due to:
- Localization noise and uncertainty
- Dynamic obstacle interactions  
- Stochastic planning algorithms
- Sensor noise and environmental variations

**Use Cases Requiring Repetitions**:
- **Regression Testing**: Ensure navigation performance doesn't degrade after code changes
- **Performance Benchmarking**: Compare different navigation configurations statistically  
- **Reliability Assessment**: Measure success rates under various conditions
- **Algorithm Validation**: Validate new planners/controllers with statistical significance

### Comprehensive Performance Metrics

#### Navigation Success Metrics
- **Test Status**: PASS/FAIL validation based on goal achievement within distance threshold
- **Nav2 Status**: Detailed navigation stack result codes (SUCCEEDED, ABORTED, CANCELED, etc.)
- **Success Rate**: Percentage of successful completions across multiple runs (critical for reliability)
- **Recovery Behaviors**: Count of recovery attempts indicating navigation difficulty

#### Timing & Performance Analysis  
- **Navigation Duration**: Total time from goal submission to completion
- **Statistical Analysis**: Mean, minimum, maximum, and standard deviation across repetitions
- **Timeout Detection**: Identification of navigation failures due to time limits

#### Path Quality & Efficiency Metrics
- **Path Efficiency**: Ratio of optimal (straight-line) distance to actual path length
- **Path Deviation**: Average distance between planned and actual trajectory points
- **Path Length Analysis**: Comparison of planned vs actual distances traveled
- **Trajectory Smoothness**: Assessment of path following accuracy and control quality

#### Advanced Analysis
- **Path Interpolation Matching**: For each planned waypoint, finds closest actual pose for precise deviation calculation
- **Multi-Run Deviation Averaging**: Statistical path analysis across all successful test iterations
- **Reference Path Extraction**: Captures complete planned and actual trajectories for visualization

## 📈 Test Reports & Visualization Data

The system generates detailed test reports in YAML format with complete trajectory data:
### Individual Run Report (Complete Navigation Analysis)
test_name: navigation_test_run1
start: {x: 0.0, y: 0.0, z: 0.0}
goal: {x: 3.0, y: 2.0, z: 0.0}
test_status: PASS
nav2_status: SUCCEEDED
duration: 15.4
path_metrics:
  planned_length_m: 3.8
  actual_length_m: 4.1
  optimal_length_m: 3.6
  actual_efficiency: 0.878
planned_path: 
  - {x: 0.0, y: 0.0, z: 0.0}
  - {x: 1.5, y: 1.0, z: 0.0}
  # ... complete planned path
actual_path:
  - {x: 0.0, y: 0.0, z: 0.0}  
  - {x: 1.4, y: 0.9, z: 0.0}
  # ... complete actual path
```

### Statistical Summary Report (Multi-Run Analysis)
```yaml
test_name: navigation_test
test_configuration:
  repetitions: 5
  success_rate_percent: 100.0
timing_statistics:
  average_time_s: 14.2
  min_time_s: 12.8
  max_time_s: 16.1
path_performance:
  avg_actual_efficiency: 0.845
  avg_path_deviation: 0.08
reference_paths:
  planned_path_full: [...]  # Complete reference paths
  actual_path_full: [...]   # for visualization
```

## 🎮 Manual Robot Control

For manual testing and verification:

```bash
# Launch just the simulation
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py

# In another terminal: Manual control
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

## 🔧 Configuration Parameters

### Position Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `start` | "0.0,0.0" | Start position "x,y" or "x,y,z,yaw" |
| `goal` | "2.0,2.0" | Goal position "x,y" or "x,y,z,yaw" |
| `start_x`, `start_y`, `start_yaw` | 0.0, 0.0, 0.0 | Individual coordinates |
| `goal_x`, `goal_y`, `goal_yaw` | 2.0, 2.0, 1.0 | Individual coordinates |

### Test Configuration
| Parameter | Default | Description |
|-----------|---------|-------------|
| `test_name` | test_01 | Test identifier for reports |
| `repetitions` | 1 | Number of test iterations |
| `dist_thres` | 0.30 | Goal completion distance (meters) |
| `world_name` | depot | Gazebo world (depot/warehouse/maze) |

## 📈 Performance Metrics

The testing suite captures comprehensive navigation performance data across all repetitions:

### Success & Reliability Metrics
- **Test Status**: PASS/FAIL based on goal achievement within distance threshold
- **Nav2 Status**: Navigation stack result codes with detailed failure analysis
- **Success Rate**: Critical reliability metric - percentage across multiple runs  
- **Recovery Count**: Number of recovery behaviors indicating navigation complexity

### Statistical Timing Analysis
- **Navigation Duration**: Total time to complete each navigation task
- **Multi-Run Statistics**: Mean, min, max, standard deviation for performance consistency
- **Performance Trends**: Identify timing patterns across repetitions

### Path Quality Assessment  
- **Path Efficiency**: Actual path vs optimal straight-line distance ratio
- **Path Deviation**: Average distance between planned and actual trajectories
- **Path Length Comparison**: Planned vs actual distances for execution analysis
- **Trajectory Smoothness**: Path following accuracy and control system performance

## 🐛 Troubleshooting

### Common Issues

**Container won't start with GUI:**
```bash
# Ensure X11 forwarding is working
xhost +local:docker
```

**Gazebo/RViz loading issues:**
```bash
# Close all instances and restart container
# Sometimes takes 30s-1min to fully load - be patient
```

**Robot teleportation fails:**
```bash
# Check Gazebo service availability
gz service -l | grep set_pose
```

**Navigation timeout:**
```bash
# Increase distance threshold for complex environments
# Verify goal position is reachable in RViz
```

### Debug Commands

```bash
# Monitor navigation progress
ros2 topic echo /navigate_to_pose/_action/feedback

# Check robot localization
ros2 topic echo /amcl_pose

# View planned path
ros2 topic echo /plan
```

## 🔬 Technical Implementation & Use Cases

### Data Collection Strategy

**Industry-Standard Approach**: 
- **Reference Path**: `/plan` topic captures Nav2 global planner intentions
- **Robot Path**: `/amcl_pose` topic tracks actual localization-based movement
- **Real-time Metrics**: Nav2 action feedback provides duration, recoveries, and completion status

### Primary Use Cases

**🔄 Continuous Integration Testing**:
- Automated regression testing after navigation stack updates
- Statistical validation with 10+ repetitions for reliable CI/CD pipelines

**⚡ Performance Benchmarking**:
- Compare different planner configurations (DWB, TEB, etc.)
- Evaluate navigation performance across various environment complexities

**🎯 Algorithm Validation**:
- Validate new navigation algorithms with statistical significance
- A/B testing of navigation parameter tunings

**🏭 Production Readiness Assessment**:
- Reliability testing with hundreds of repetitions
- Success rate validation for deployment decisions

### Auto-World Detection

The system can automatically detect the active Gazebo world or use specified world parameters, enabling seamless integration with existing simulation environments.

### Path Deviation Analysis Algorithm

## 📁 Repository Structure

```
turtlebot4-navigation-testing/
├── Dockerfile                 # ROS 2 Jazzy + TurtleBot4 environment
├── build_docker.sh           # Docker build script  
├── run_docker.sh             # Container launch script
├── src/nav2_performance_tests/
│   ├── launch/
│   │   └── nav2_test_suite.launch.py
│   ├── nav2_performance_tests/
│   │   ├── nav2_test_node.py
│   │   └── amcl_pose_initializer.py
│   └── setup.py
├── reports/                   # Generated test reports
└── README.md                 # This file
```

## 🎯 Challenge Requirements Fulfilled

✅ **Dockerfile with ROS 2 Jazzy** - Complete development environment on Ubuntu 24.04  
✅ **Build/Run Scripts** - `build_docker.sh` and `run_docker.sh`  
✅ **Robot Simulation** - TurtleBot4 in Gazebo with multiple worlds  
✅ **Teleoperation Interface** - Keyboard control integration  
✅ **Automated Navigation Test** - Point A to B navigation with success validation  
✅ **Comprehensive Reports** - YAML reports with planned vs actual paths  
✅ **Path Visualization Data** - Complete trajectory data for analysis  

## 🚀 Advanced Usage

### Batch Testing
Create custom test configurations with multiple navigation scenarios:

```bash
# Example: Run comprehensive warehouse testing
ros2 launch nav2_performance_tests nav2_test_suite.launch.py \
    world_name:=warehouse \
    test_file:=config/warehouse_test_suite.yaml
```

## 📞 Support

If you encounter issues:

1. Check the troubleshooting section above
2. Review generated log files in the reports directory  
3. Ensure Docker has sufficient resources allocated
4. Verify all prerequisites are properly installed

---

**Built with ROS 2 Jazzy Jellyfish 🤖 | TurtleBot4 🐢 | Nav2 🧭**