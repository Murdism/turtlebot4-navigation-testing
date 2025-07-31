# TurtleBot4 Navigation Testing Suite
A comprehensive ROS 2 testing framework for evaluating TurtleBot4 navigation (Nav2) performance in Gazebo simulation environments. This suite provides automated navigation testing with detailed performance metrics, path analysis, and flexible parameter configuration.

## Overview

This testing suite consists of three main components:
1. **Launch File** - Configurable test orchestration
2. **Nav2 Test Action Server** - Core navigation testing logic
3. **Initial Pose Publisher** - AMCL pose initialization utility

## Features

- **Flexible Parameter Input**: Support for both individual coordinates and comma-separated format
- **Dynamic World/Map Selection**: Automatically matches world names with corresponding map files
- **Comprehensive Test Reports**: YAML reports with detailed metrics and path analysis
- **Multi-run Testing**: Support for repeated test execution with statistical analysis
- **Robust State Management**: Proper robot teleportation, pose initialization, and Nav2 state handling
- **Path Deviation Analysis**: Comparison between planned and actual robot trajectories

## Components

### 1. Launch File (`turtlebot4_nav_test.launch.py`)

The main orchestration component that:
- Launches TurtleBot4 Gazebo simulation
- Configures Nav2 stack with localization
- Starts the navigation test action server
- Handles dynamic parameter parsing

### 2. Nav2 Test Action Server (`nav2_test_action_server.py`)

Core testing engine that:
- Manages robot teleportation in Gazebo
- Handles AMCL pose initialization
- Executes navigation goals with timeout handling
- Collects performance metrics and path data
- Generates detailed test reports

### 3. Initial Pose Publisher (`initial_pose_publisher.py`)

Utility for reliable AMCL initialization:
- Publishes initial pose to AMCL
- Waits for pose acceptance with covariance validation
- Handles timeout scenarios gracefully

## Parameters

### Start Position Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `start_x` | 0.0 | Start X coordinate (meters) |
| `start_y` | 0.0 | Start Y coordinate (meters) |
| `start_z` | 0.0 | Start Z coordinate (meters) |
| `start_yaw` | 0.0 | Start yaw orientation (radians) |

### Goal Position Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `goal_x` | 2.0 | Goal X coordinate (meters) |
| `goal_y` | 2.0 | Goal Y coordinate (meters) |
| `goal_z` | 0.0 | Goal Z coordinate (meters) |
| `goal_yaw` | 1.0 | Goal yaw orientation (radians) |

### Combined Format Parameters (Alternative)
| Parameter | Format | Description |
|-----------|--------|-------------|
| `start` | "x,y" or "x,y,z,yaw" | Combined start position |
| `goal` | "x,y" or "x,y,z,yaw" | Combined goal position |

### Test Configuration Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `dist_thres` | 0.30 | Distance threshold for goal completion (meters) |
| `test_name` | test_01 | Test identifier for report naming |
| `repetitions` | 1 | Number of test repetitions |
| `world_name` | depot | Gazebo world environment |
| `entity_name` | turtlebot4 | Robot entity name in Gazebo |



📊 Data Source Selection & Rationale
Reference Path: /plan Topic

Source: Nav2 global planner output
Rationale: Represents the intended navigation path as computed by the planning algorithm
Industry Standard: This is the canonical reference for Nav2 performance analysis
Benefits: Shows planning quality, path smoothness, and obstacle avoidance strategy

Robot Path: /amcl_pose Topic

Source: AMCL (Adaptive Monte Carlo Localization) pose estimates
Rationale: Represents where the robot believes it is during navigation
Industry Standard: Used in production systems for performance monitoring
Benefits:

Captures real-world localization uncertainty
Works on both simulation and physical robots
Reflects actual navigation system behavior
Enables detection of both planning and localization issues
## Usage Examples

### Basic Navigation Test
world options: depot, maze, warehouse
```bash
# Simple test with default depot world
ros2 launch nav2_tests turtlebot4_nav_test.launch.py \
    start:="0.0,0.0" goal:="3.0,2.0" 
```

### Custom World Environment

```bash
# Test in warehouse environment
ros2 launch nav2_tests turtlebot4_nav_test.launch.py \
    world_name:=warehouse \
    start:="1.0,1.0,0.0,0.0" goal:="8.0,5.0,0.0,1.57" \
    test_name:=warehouse_test
```

### Multi-Run Performance Testing

```bash
# Run 10 repetitions for statistical analysis
ros2 launch nav2_tests turtlebot4_nav_test.launch.py \
    start:="0.5,0.5" goal:="4.5,3.5" \
    repetitions:=10 \
    test_name:=performance_analysis \
    dist_thres:=0.15
```

### Individual Parameter Format

```bash
# Using individual coordinate parameters
ros2 launch nav2_tests turtlebot4_nav_test.launch.py \
    start_x:=1.0 start_y:=3.0 start_yaw:=0.785 \
    goal_x:=5.0 goal_y:=3.0 goal_yaw:=0.0 \
    world_name:='warehouse'  repetitions:=2
```

### Precision Testing

```bash
# High precision test with tight tolerance
ros2 launch nav2_tests turtlebot4_nav_test.launch.py \
    start:="2.0,1.0" goal:="6.0,4.0" \
    dist_thres:=0.10 \
    repetitions:=5 \
    test_name:=precision_test
```

## Test Reports

The system generates two types of reports:

### Individual Run Reports (`{test_name}_run{N}_result.yaml`)

Contains detailed metrics for each test run:

```yaml
test_name: navigation_test_run1
start: {x: 0.0, y: 0.0, z: 0.0}
goal: {x: 3.0, y: 2.0, z: 0.0}
test_status: PASS
nav2_status: SUCCEEDED
number_of_recoveries: 0
remaining_distance: 0.12
duration: 15.4
planned_path: 
  - {x: 0.0, y: 0.0, z: 0.0}
  - {x: 1.5, y: 1.0, z: 0.0}
  # ... full planned path
actual_path:
  - {x: 0.0, y: 0.0, z: 0.0}
  - {x: 1.4, y: 0.9, z: 0.0}
  # ... full actual path
```

### Summary Reports (`{test_name}_summary.yaml`)

Aggregated statistics across all runs:

```yaml
test_name: navigation_test
repetitions: 5
average_time: 14.2
min_time: 12.8
max_time: 16.1
success_rate: 1.0
average_deviation: 0.08
runs:
  - test_name: navigation_test_run1
    start: {x: 0.0, y: 0.0}
    goal: {x: 3.0, y: 2.0}
    status: PASS
    duration: 15.4
    detail_file: reports/navigation_test_run1_result.yaml
  # ... additional runs
```

## Test Metrics

### Performance Metrics
- **Duration**: Total navigation time (seconds)
- **Success Rate**: Percentage of successful completions
- **Recovery Count**: Number of recovery behaviors triggered
- **Remaining Distance**: Final distance to goal (meters)

### Path Analysis
- **Average Deviation**: Mean distance between planned and actual path points
- **Path Efficiency**: Comparison of planned vs actual path lengths
- **Trajectory Smoothness**: Analysis of path following accuracy

## Architecture

### Test Flow
1. **Initialization**: Launch Gazebo simulation and Nav2 stack
2. **Robot Positioning**: Teleport robot to start position in Gazebo
3. **Pose Setting**: Initialize AMCL with start pose
4. **Navigation**: Send goal and monitor progress
5. **Data Collection**: Record metrics, paths, and performance data
6. **Reporting**: Generate detailed YAML reports
7. **Repetition**: Repeat for multiple runs if specified

### State Management
- **Gazebo Teleportation**: Uses `gz service` for precise robot positioning
- **AMCL Initialization**: Reliable pose setting with covariance validation
- **Nav2 State Monitoring**: Ensures navigation stack readiness
- **Costmap Clearing**: Fresh state between test runs

## Error Handling

### Common Issues and Solutions

**Robot teleportation fails:**
```bash
# Check Gazebo service availability
gz service -l | grep set_pose
```

**AMCL pose not accepted:**
- Verify map file exists and matches world
- Check initial pose is within map bounds
- Increase covariance threshold if needed

**Navigation timeout:**
- Increase distance threshold for complex environments
- Verify goal position is reachable
- Check for obstacles blocking path

### Debug Commands

```bash
# Monitor navigation topics
ros2 topic echo /navigate_to_pose/_action/feedback

# Check AMCL pose
ros2 topic echo /amcl_pose

# Verify costmaps
ros2 run nav2_costmap_2d nav2_costmap_2d_markers voxel_grid:=/local_costmap/voxel_grid visualization_marker:=/my_marker
```

## Requirements

### System Dependencies
- ROS 2 Jazzy
- Gazebo Garden/Harmonic
- Python 3.8+

### ROS 2 Packages
- `turtlebot4_gz_bringup` - TurtleBot4 Gazebo simulation
- `turtlebot4_navigation` - Navigation maps and configurations
- `nav2_msgs` - Navigation action interfaces
- `geometry_msgs` - Pose and geometry messages
- `lifecycle_msgs` - Node lifecycle management

### Python Dependencies
```bash
pip install PyYAML
```

## Installation

1. **Clone the repository:**
```bash
cd ~/ros2_ws/src
git clone <repository_url> nav2_tests
```

2. **Build the package:**
```bash
cd ~/ros2_ws
colcon build --packages-select nav2_tests
source install/setup.bash
```

3. **Verify TurtleBot4 packages:**
```bash
ros2 pkg list | grep turtlebot4
```

## Advanced Usage

### Custom World Creation

1. Create world file in `turtlebot4_gz_bringup/worlds/`
2. Create corresponding map file in `turtlebot4_navigation/maps/`
3. Use `world_name` parameter to specify custom world

### Test Automation

```python
# Example test suite script
test_configs = [
    {"start": "0,0", "goal": "3,2", "world": "depot"},
    {"start": "1,1", "goal": "5,4", "world": "warehouse"},
    {"start": "2,0", "goal": "4,3", "world": "office"}
]

for config in test_configs:
    subprocess.run([
        "ros2", "launch", "nav2_tests", "turtlebot4_nav_test.launch.py",
        f"start:={config['start']}", 
        f"goal:={config['goal']}",
        f"world_name:={config['world']}",
        "repetitions:=5"
    ])
```

### Performance Benchmarking

For systematic performance evaluation:

```bash
# Create benchmark script
#!/bin/bash
WORLDS=("depot" "warehouse" "office")
DISTANCES=("short:1,1,3,2" "medium:0,0,5,4" "long:0,0,8,6")

for world in "${WORLDS[@]}"; do
    for dist in "${DISTANCES[@]}"; do
        IFS=':' read -r name coords <<< "$dist"
        ros2 launch nav2_tests turtlebot4_nav_test.launch.py \
            world_name:=$world \
            start:="0,0" goal:="$coords" \
            test_name:="${world}_${name}" \
            repetitions:=10
    done
done
```

## Troubleshooting

### Launch Issues
- Ensure all TurtleBot4 packages are properly installed
- Verify Gazebo and ROS 2 versions compatibility
- Check that required topics and services are available

### Test Failures
- Review generated YAML reports for detailed error information
- Monitor ROS 2 logs during test execution
- Verify map and world file consistency

### Performance Issues
- Reduce repetitions for initial testing
- Use smaller distance thresholds for precision requirements
- Consider environment complexity when setting timeouts

