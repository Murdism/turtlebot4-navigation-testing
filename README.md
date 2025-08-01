# 🐢 TurtleBot4 Navigation Testing Suite


## 🎥 Demo

**Turtle Bot Batch Testing**

![Watch TurtleBot4 Navigation Demo](./demo/Batch_test_turtle_bot_ultra.gif)

📹 **[Complete Demo Video Collection](https://drive.google.com/drive/folders/1-TDA9TRJ-gcsTLjjDPdYjRyHc5yL2MIe?usp=sharing)**

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

## 🐢 Quick Start

### 📹 See It In Action First
Before diving into setup, check out the [live demonstration videos](https://drive.google.com/drive/folders/1-TDA9TRJ-gcsTLjjDPdYjRyHc5yL2MIe?usp=drive_link) showing the complete navigation testing suite in operation.

*Featuring: Single navigation tests, multi-run analysis, batch testing, path efficiency analysis, and automated reporting*

### Prerequisites

- **Docker** (with NVIDIA Container Toolkit for GPU support)
- **Git**
- **Network connection** - Required for downloading Gazebo worlds and ROS packages
- **System Requirements**: At least 8GB RAM and 4 CPU cores recommended
- **Display**: X11 forwarding capability for GUI applications (RViz, Gazebo)
- **Performance**: Close unnecessary applications to ensure smooth simulation

### 1. Clone Repository

```bash
git clone https://github.com/Murdism/turtlebot4-navigation-testing.git
cd turtlebot4-navigation-testing
```

### 2. Build Docker Environment

#### Requirements for Docker with GPU Support
```bash
# Install NVIDIA Container Toolkit for GPU acceleration
sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo systemctl restart docker

# Verify GPU support (optional)
nvidia-smi  # Should show GPU information
```

#### Build the Container
```bash
# Make scripts executable
chmod +x docker/build_docker.sh docker/run_docker.sh

# Build the Docker image (includes ROS 2 Jazzy + TurtleBot4)
# Note: First build may take few minutes to download dependencies
./docker/build_docker.sh
```

### 3. Run the Container

```bash
# Start the container with GUI support (persistent by default)
# Note: Ensure Docker has adequate resources allocated (8GB+ RAM, 4+ CPU cores)
./docker/run_docker.sh

# For temporary containers that are removed after exit
./docker/run_docker.sh --rm

# Custom resource allocation
./docker/run_docker.sh --memory 8g --cpus 4
```

**Container Options:**
- **Default**: Container persists after exit (can be restarted with `docker start ros2_nav2_container`)
- **`--rm`**: Remove container automatically after exit (useful for clean testing)
- **Resource flags**: `--memory`, `--cpus`, `--shm-size` for custom allocation
- **GPU support**: Auto-detected, or use `--gpu true/false` to override

**Troubleshooting GUI Issues:**
```bash
# If GUI applications don't display, enable X11 forwarding
xhost +local:docker
./docker/run_docker.sh
```
**Troubleshooting GUI Issues:**
```bash
# If GUI applications don't display, enable X11 forwarding
xhost +local:docker
./docker/run_docker.sh
```

### 4. Open Additional Container Terminals

To open additional terminals inside the running container:
```bash
docker exec -it ros2_nav2_container bash
# Inside the container, always source the workspace
source install/setup.bash
```

### 5. Build and Source the Workspace

Inside the container:
```bash
colcon build --symlink-install
source install/setup.bash
```

## 🧪 Testing Capabilities

### Available Test Worlds

- **depot** - Simple warehouse environment (default, fastest loading)
- **warehouse** - Complex multi-room facility with obstacles
- **maze** - Challenging maze navigation for advanced testing

### 🔺 Available Navigation Options

You can run:

1. **Manual Teleop Tests**: Manually control the robot using keyboard or Gazebo GUI sliders
2. **Single Run Test**: Navigate from a fixed location to a goal once
3. **Repetitive Runs**: Repeat the same test multiple times to analyze consistency and performance
4. **Batch Tests**: Run a YAML-defined suite of different start-goal navigation tests
5. **Sequential Live Runs**: Run additional tests without restarting Gazebo

## 🎮 Basic Testing Examples

📹 **See these examples in action**: [Demo Video Collection](https://drive.google.com/drive/folders/1-TDA9TRJ-gcsTLjjDPdYjRyHc5yL2MIe?usp=drive_link)

###  Simulation Test
Test if Gazebo and TurtleBot4 are working correctly:
```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py 
```

**Important Setup Notes:**
- **Before moving the robot**: Go to Gazebo, select the robot, and click "undock"
- **First Launch**: Initial startup may take 30s-1min as Gazebo downloads world assets
- **Network Issues**: If Gazebo shows "Requesting list of worlds" or loads very slowly, check your internet connection as it downloads world files on first run

**Troubleshooting First Launch:**
```bash
# If Gazebo hangs on world loading
# 1. Check network connectivity
ping gazebosim.org

# 2. Monitor download progress
docker logs ros2_nav2_container

# 3. If needed, restart with fresh container
docker stop ros2_nav2_container
./docker/run_docker.sh
```

### Manual Robot Control
For manual testing and verification:
```bash
# In a new terminal (inside container)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

**Alternative**: You can control the robot directly from Gazebo using keyboard controls or GUI sliders.

### Basic Navigation Test
For automated navigation testing using available maps (AMCL for localization):

⚠️ **Important**: Make sure all RViz and Gazebo sessions are closed before starting to avoid resource conflicts.

```bash
source install/setup.bash
# Basic navigation test (depot world: 0,0 → 8,7)
ros2 launch nav2_performance_tests nav2_test_suite.launch.py start:="0.0,0.0" goal:="8.0,7.0" repetitions:=2 dist_thres:=0.25
```

**Notes**: 
- First-time loading might take 30s-1min - be patient
- If RViz doesn't load the map, close all instances and restart
- Check results: `ls reports/` and `cat reports/test_01_summary.yaml` (reports generated in current directory)

## 🔬 Navigation Testing Scenarios

### Important: Clean Environment Setup

**Note**: If you are running the launch file multiple times or have closed and restarted sessions, it's recommended to ensure there are no leftover instances to avoid conflicts:

```bash
# Clean up any existing processes before starting new tests
pkill -f rviz
pkill -f gazebo  
pkill -f ros2
```

**When to use cleanup:**
- Between test sessions (if relaunched)
- After interrupted or failed launches
- When seeing TF_OLD_DATA or RTPS_TRANSPORT_SHM errors
- Before running different world environments

### Single Navigation Test

```bash
# Simple point-to-point navigation
ros2 launch nav2_performance_tests nav2_test_suite.launch.py \
    start:="0.0,0.0" goal:="4.0,3.0" \
    test_name:=basic_nav_test
```

### Multi-Run Performance Testing

```bash
# Run 5 iterations for statistical analysis
ros2 launch nav2_performance_tests nav2_test_suite.launch.py \
    start:="0.5,0.5" goal:="4.5,3.5" \
    repetitions:=5 \
    test_name:=performance_analysis \
    dist_thres:=0.15
```

### Sequential Testing (Without Restarting Simulation)

After running your initial test via the launch file, you can run additional tests using just the node without restarting the entire simulation environment:

```bash
# Open new container terminal
docker exec -it ros2_nav2_container bash
source install/setup.bash

# Run another test without restarting simulation
ros2 run nav2_performance_tests Nav2TestNode --ros-args \
  -p start_x:=2.0 \
  -p start_y:=1.0 \
  -p goal_x:=6.0 \
  -p goal_y:=4.0 \
  -p test_name:=sequential_test_2
```

### Different World Environments

```bash
# Test in warehouse environment
ros2 launch nav2_performance_tests nav2_test_suite.launch.py \
    world_name:=warehouse \
    start:="1.0,1.0,0.0,0.0" goal:="8.0,5.0,0.0,1.57" \
    test_name:=warehouse_test
```

## 🐢 Advanced Usage

### Batch Testing
Create custom test configurations with multiple navigation scenarios:

```bash
# Example: Run comprehensive warehouse testing
ros2 launch nav2_performance_tests nav2_test_suite.launch.py \
    world_name:=warehouse \
    test_file:=config/warehouse_test_suite.yaml
```

### Custom Test Configuration Files
Create YAML files for complex test suites:

```yaml
# Example: config/warehouse_tests.yaml
world_name: "warehouse"
nav2: true
rviz: true
slam: false
localization: true

tests:
  - test_name: "warehouse_corner_to_center"
    start_x: 8.0
    start_y: 7.0
    start_z: 0.0
    start_yaw: 0.0
    goal_x: 1.0
    goal_y: 1.0
    goal_z: 0.0
    goal_yaw: 1.57
    repetitions: 1
    dist_thres: 0.3

  - test_name: "warehouse_center_to_corner"
    start_x: 1.0
    start_y: 1.0
    start_z: 0.0
    start_yaw: 1.57
    goal_x: 8.0    
    goal_y: 7.0
    goal_z: 0.0
    goal_yaw: 0.0
    repetitions: 2
    dist_thres: 0.3
```
## 📋 Parameter Configuration Options

### Launch File Parameters

All parameters can be passed via `ros2 launch`:

| Parameter | Format | Example | Description |
|-----------|--------|---------|-------------|
| `start` | "x,y" or "x,y,z,yaw" | "1.0,2.0" or "1.0,2.0,0.0,1.57" | Start position |
| `goal` | "x,y" or "x,y,z,yaw" | "4.0,3.0" or "4.0,3.0,0.0,0.0" | Goal position |
| `start_x`, `start_y`, `start_z`, `start_yaw` | Individual floats | `start_x:=1.0` | Individual start coordinates |
| `goal_x`, `goal_y`, `goal_z`, `goal_yaw` | Individual floats | `goal_x:=4.0` | Individual goal coordinates |
| `dist_thres` | Float (meters) | `dist_thres:=0.30` | Success distance threshold |
| `test_name` | String | `test_name:=my_test` | Custom test identifier |
| `repetitions` | Integer | `repetitions:=5` | Number of test repetitions |
| `world_name` | String | `world_name:=warehouse` | Gazebo world to use |
| `entity_name` | String | `entity_name:=turtlebot4` | Robot name in Gazebo |
| `test_file` | Path | `test_file:=config/tests.yaml` | Batch test configuration file |

### Nav2TestNode Parameters

When running the node directly with `ros2 run`:

```bash
ros2 run nav2_performance_tests Nav2TestNode --ros-args \
  -p start_x:=1.0 \
  -p start_y:=2.0 \
  -p goal_x:=4.0 \
  -p goal_y:=3.0 \
  -p test_name:=direct_test \
  -p repetitions:=3
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
```yaml
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

## 🐛 Troubleshooting

### Common Issues & Solutions

#### Container and GUI Issues
**Container won't start with GUI:**
```bash
# Ensure X11 forwarding is working
xhost +local:docker
```

**Permission denied errors:**
```bash
# Fix Docker permissions
sudo chmod +x docker/build_docker.sh docker/run_docker.sh
```

#### Simulation Loading Issues
**Gazebo/RViz loading issues:**
```bash
# Close all instances and restart container
# Sometimes takes 30s-1min to fully load - be patient
```

**RViz doesn't show map:**
- Close all RViz instances completely
- Restart the container
- Wait for full Gazebo loading before expecting RViz to work

#### Navigation Issues
**Robot teleportation fails:**
```bash
# Check Gazebo service availability
gz service -l | grep set_pose
```

**Navigation timeout or fails:**
```bash
# Increase distance threshold for complex environments
# Verify goal position is reachable in RViz
# Check that the goal is not inside an obstacle
```

**Parameter parsing errors:**
```bash
# Use correct parameter format
# Correct:   start:="1.0,2.0"
# Incorrect: start:"1.0,2.0" (missing =)
```

### Debug Commands

```bash
# Monitor navigation progress
ros2 topic echo /navigate_to_pose/_action/feedback

# Check robot localization
ros2 topic echo /amcl_pose

# View planned path
ros2 topic echo /plan

# Check available services
ros2 service list | grep nav

# Check node status
ros2 node list
ros2 node info /Nav2TestNode
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
- Evaluate navigation performance across various environment complexities

**🎯 Algorithm Validation**:
- Validate new navigation algorithms with statistical significance
- A/B testing of navigation parameter tunings

**🏭 Production Readiness Assessment**:
- Reliability testing with hundreds of repetitions
- Success rate validation for deployment decisions

### Auto-World Detection

The system can automatically detect the active Gazebo world or use specified world parameters, enabling seamless integration with existing simulation environments.

## 📁 Repository Structure

turtlebot4-navigation-testing/
├── Dockerfile                    # ROS 2 Jazzy + TurtleBot4 environment
├── demo/
│   └── Simple_scnario.gif        # Navigation demonstration
├── docker/
│   ├── build_docker.sh          # Docker build script  
│   └── run_docker.sh            # Container launch script with dynamic resources
├── ros2_ws/
│   └── src/nav2_performance_tests/
│       ├── launch/
│       │   └── nav2_test_suite.launch.py
│       ├── nav2_performance_tests/
│       │   ├── __init__.py       # Package initialization
│       │   ├── nav2_test_node.py # Main navigation test engine
│       │   ├── navigation_test_error_handler.py # Error handling and troubleshooting
│       │   └── amcl_pose_initializer.py # Robot pose initialization
│       ├── setup.py              # Package configuration
│       └── package.xml           # ROS package metadata
├── config/                       # Configuration files (optional)
│   └── example_batch_tests.yaml  # Sample batch test configuration
├── reports/                      # Generated test reports (auto-created in working directory)
└── README.md                     # This documentation

## 🎯 Challenge Requirements Fulfilled

✅ **Dockerfile with ROS 2 Jazzy** - Complete development environment on Ubuntu 24.04  
✅ **Build/Run Scripts** - `build_docker.sh` and `run_docker.sh`  
✅ **Robot Simulation** - TurtleBot4 in Gazebo with multiple worlds  
✅ **Teleoperation Interface** - Keyboard control integration  
✅ **Automated Navigation Test** - Point A to B navigation with success validation  
✅ **Comprehensive Reports** - YAML reports with planned vs actual paths  
✅ **Path Visualization Data** - Complete trajectory data for analysis  

### 📹 Live Proof of Implementation
All features demonstrated in action: **[Video Collection](https://drive.google.com/drive/folders/1-TDA9TRJ-gcsTLjjDPdYjRyHc5yL2MIe?usp=drive_link)**

## 🐢 Advanced Usage

### Batch Testing
Create custom test configurations with multiple navigation scenarios:

```bash
# Example: Run comprehensive warehouse testing
ros2 launch nav2_performance_tests nav2_test_suite.launch.py \
    world_name:=warehouse \
    test_file:=config/warehouse_test_suite.yaml
```

### Custom Test Configuration Files
Create YAML files for complex test suites:

```yaml
# Example: config/warehouse_tests.yaml
world_name: warehouse
entity_name: turtlebot4
tests:
  - test_name: "short_path"
    start_x: 1.0
    start_y: 1.0
    goal_x: 3.0
    goal_y: 2.0
    repetitions: 3
  - test_name: "long_path"
    start_x: 0.0
    start_y: 0.0
    goal_x: 8.0
    goal_y: 6.0
    repetitions: 5
    dist_thres: 0.2
```

## 📞 Support

If you encounter issues:

1. **Check the troubleshooting section** above for common solutions
2. **Review generated log files** in the reports directory (created in your current working directory)  
3. **Ensure Docker has sufficient resources** allocated (8GB+ RAM recommended)
4. **Verify all prerequisites** are properly installed
5. **Check container logs**: `docker logs ros2_nav2_container`

### Getting Help

- Ensure your goal positions are reachable and not inside obstacles
- Use RViz to visualize the map and verify your start/goal positions
- Start with simple, short-distance tests before attempting complex navigation
- Monitor system resources - Gazebo can be resource-intensive

---

**Built with ROS 2 Jazzy Jellyfish 🤖 | TurtleBot4 🐢 | Nav2 🧭**

*A complete ROS 2-based test automation framework to validate and evaluate TurtleBot4 navigation performance in Gazebo. Built for rigorous navigation testing with statistical analysis and comprehensive reporting.*
