import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from nav_msgs.msg import Path
from lifecycle_msgs.srv import GetState
import yaml
import os
import time
import subprocess
import math
from datetime import datetime
import re

class Nav2TestNode(Node):
    STATUS_CODE_TO_STRING = {
        0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
        4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'
    }

    def __init__(self):
        super().__init__('Nav2TestNode')
        # Parameters for start/goal poses, thresholds, repetitions, etc.
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_z', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_z', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('test_name', 'test_01')
        self.declare_parameter('dist_thres', 0.30)
        self.declare_parameter('repetitions', 1)
        self.declare_parameter('world_name', 'depot')
        self.declare_parameter('entity_name', 'turtlebot4')
        # ADD: Parameter to auto-detect world or use specified world
        self.declare_parameter('auto_detect_world', True)
        self.declare_parameter('test_file', '')  # Optional test file for custom tests
        # Compound parameters
        self.declare_parameter('start', '')  # Format: "x,y" or "x,y,z,yaw"
        self.declare_parameter('goal', '')   # Format: "x,y" or "x,y,z,yaw"
        
        # Parse compound parameters if provided
        self._parse_compound_parameters()
        
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.result_pub = self.create_publisher(String, 'nav2_test_result', 10)
                # Service for running individual tests

        self.planned_path = []
        self.actual_path = []
        self._feedback_data = []

        self.plan_sub = self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.odom_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.odom_callback, 10)

        self.run_durations = []
        self.run_results = []
        self.per_run_summaries = []
        
        # Track current pose for validation
        self.current_pose = None
        
        # ADD: Track initial plan capture state
        self.initial_plan_captured = False

    def plan_callback(self, msg):
        """Capture the first complete planned path from the /plan topic
        """
        if not self.initial_plan_captured and msg.poses:
            # Only capture the first complete plan - this is the full path from start to goal
            self.planned_path = []
            for pose_stamped in msg.poses:
                self.planned_path.append({
                    'x': pose_stamped.pose.position.x,
                    'y': pose_stamped.pose.position.y,
                    'z': pose_stamped.pose.position.z,
                })
            
            self.initial_plan_captured = True
            self.get_logger().info(f"✅ Captured initial planned path with {len(self.planned_path)} waypoints "
                                 f"from ({self.planned_path[0]['x']:.2f}, {self.planned_path[0]['y']:.2f}) "
                                 f"to ({self.planned_path[-1]['x']:.2f}, {self.planned_path[-1]['y']:.2f})")
        

    def detect_active_world(self):
        """
        AUTO-DETECT the currently running Gazebo world
        
        Returns: world_name if found, None if no world detected
        """
        try:
            # List all available Gazebo services
            result = subprocess.check_output(["gz", "service", "-l"], timeout=5).decode()
            
            # Look for world services (format: /world/{world_name}/set_pose)
            world_pattern = r'/world/([^/]+)/set_pose'
            matches = re.findall(world_pattern, result)
            
            if matches:
                active_world = matches[0]  # Take the first world found
                self.get_logger().info(f"🌍 Auto-detected active world: '{active_world}'")
                return active_world
            else:
                self.get_logger().warn("⚠️  No active Gazebo worlds detected")
                return None
                
        except subprocess.TimeoutExpired:
            self.get_logger().error("❌ Timeout detecting Gazebo world")
            return None
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"❌ Failed to detect Gazebo world: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"❌ Error detecting world: {e}")
            return None

    def get_world_name(self):
        """
        Get world name: auto-detect if enabled, otherwise use parameter
        
        Returns: world_name to use for teleportation
        """
        auto_detect = self.get_parameter('auto_detect_world').value
        
        if auto_detect:
            detected_world = self.detect_active_world()
            if detected_world:
                return detected_world
            else:
                # Fallback to parameter if auto-detection fails
                fallback_world = self.get_parameter('world_name').value
                self.get_logger().warn(f"⚠️  Auto-detection failed, using fallback world: '{fallback_world}'")
                return fallback_world
        else:
            # Use parameter directly
            param_world = self.get_parameter('world_name').value
            self.get_logger().info(f"🌍 Using specified world: '{param_world}'")
            return param_world

    def odom_callback(self, msg):
        self.current_pose = msg  # Store current pose for validation
        self.actual_path.append({
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
        })

    def feedback_callback(self, feedback_msg):
        self._feedback_data.append(feedback_msg.feedback)

    def wait_for_gz_service(self, world_name, timeout=120):
        srv_name = f"/world/{world_name}/set_pose"
        self.get_logger().info(f"Waiting for gz service {srv_name} to be available...")
        start = time.time()
        while True:
            try:
                out = subprocess.check_output(["gz", "service", "-l"]).decode()
                if srv_name in out:
                    self.get_logger().info(f"gz service {srv_name} is now available.")
                    return True
            except Exception as e:
                self.get_logger().warn(f"gz service -l failed: {e}")
            if time.time() - start > timeout:
                self.get_logger().error(f"Timeout while waiting for gz service {srv_name}")
                raise RuntimeError(f"Timeout waiting for gz service {srv_name}")
            time.sleep(0.5)

    def teleport_robot(self, world_name, entity_name, x, y, z, yaw):
        self.wait_for_gz_service(world_name)
        qw = math.cos(yaw / 2.0)
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        gz_cmd = [
            "gz", "service", "-s", f"/world/{world_name}/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req",
            f"name: '{entity_name}', position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}"
        ]
        self.get_logger().info(f"Teleporting robot (gz): {' '.join(gz_cmd)}")
        try:
            result = subprocess.run(gz_cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            output = result.stdout
            self.get_logger().info(f"gz output: {output.strip()}")
            output_clean = output.replace('\n', '').replace('\r', '').strip().lower()
            if "true" in output_clean:
                self.get_logger().info("Teleportation succeeded.")
                return True
            else:
                self.get_logger().error(f"Teleportation failed: {output.strip()}")
                return False
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Teleportation subprocess failed: {e.stderr.strip()}")
            return False

    def wait_for_pose_update(self, expected_x, expected_y, timeout=10.0, tolerance=0.5):
        """Wait for AMCL pose to update to expected position"""
        self.get_logger().info(f"Waiting for pose update to ({expected_x:.2f}, {expected_y:.2f})")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if self.current_pose is not None:
                current_x = self.current_pose.pose.pose.position.x
                current_y = self.current_pose.pose.pose.position.y
                distance = math.sqrt((current_x - expected_x)**2 + (current_y - expected_y)**2)
                
                self.get_logger().info(f"Current pose: ({current_x:.2f}, {current_y:.2f}), distance from expected: {distance:.2f}")
                
                if distance < tolerance:
                    self.get_logger().info("Pose updated successfully!")
                    return True
            
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        self.get_logger().warn(f"Timeout waiting for pose update")
        return False

    def clear_costmaps(self):
        """Clear Nav2 costmaps to ensure clean state"""
        self.get_logger().info("Clearing Nav2 costmaps...")
        clear_services = [
            "/local_costmap/clear_entirely_local_costmap",
            "/global_costmap/clear_entirely_global_costmap"
        ]
        for srv in clear_services:
            for attempt in range(5):  
                try:
                    result = subprocess.run(
                        ["ros2", "service", "call", srv, "std_srvs/srv/Empty", "{}"],
                        check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5
                    )
                    self.get_logger().info(f"Successfully cleared {srv}")
                    break
                except subprocess.CalledProcessError as e:
                    self.get_logger().warn(f"Attempt {attempt+1}: Service {srv} failed: {e.stderr}")
                    if attempt < 4:
                        time.sleep(1.0)
                except subprocess.TimeoutExpired:
                    self.get_logger().warn(f"Timeout clearing {srv}")
                    break

    def cancel_all_goals(self):
        """Cancel any existing navigation goals"""
        try:
            # Cancel current goal if any
            self.get_logger().info("Canceling any existing navigation goals...")
            subprocess.run(
                ["ros2", "action", "send_goal", "/navigate_to_pose", "nav2_msgs/action/NavigateToPose", 
                 "{pose: {header: {frame_id: map}, pose: {position: {x: 0, y: 0, z: 0}, orientation: {w: 1}}}}",
                 "--feedback"], 
                timeout=1, capture_output=True
            )
        except:
            pass  # Ignore errors, this is just cleanup

    def wait_for_nav2_active(self):
        client = self.create_client(GetState, '/bt_navigator/get_state')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /bt_navigator/get_state service...')
        req = GetState.Request()
        while True:
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result().current_state.label == "active":
                self.get_logger().info('bt_navigator is ACTIVE.')
                break
            self.get_logger().info(f'bt_navigator state: {future.result().current_state.label}')
            time.sleep(0.5)

    def set_initial_pose(self, x, y, z, yaw):
        """Set initial pose and wait for it to be accepted"""
        self.get_logger().info(f"Setting initial pose to ({x:.2f}, {y:.2f}, {yaw:.2f})")
        
        # Method 1: Use the existing initial_pose_publisher
        cmd = [
            "ros2", "run", "nav2_performance_tests", "amcl_pose_initializer",
            "--ros-args",
            "-p", f"x:={x}",
            "-p", f"y:={y}",
            "-p", f"z:={z}",
            "-p", f"yaw:={yaw}",
            "-p", "cov_thresh:=0.5"
        ]
        
        try:
            subprocess.run(cmd, check=True, timeout=10)
            self.get_logger().info("Initial pose publisher completed")
        except subprocess.TimeoutExpired:
            self.get_logger().warn("Initial pose publisher timed out")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Initial pose publisher failed: {e}")
        
        # Wait for pose to actually update
        self.wait_for_pose_update(x, y, timeout=10.0)

    def validate_goal_distance(self, start_pose, goal_pose, min_distance=0.5):
        """Ensure goal is far enough from start to require actual movement"""
        start_x = start_pose.pose.position.x
        start_y = start_pose.pose.position.y
        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y
        
        distance = math.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2)
        self.get_logger().info(f"Distance from start to goal: {distance:.2f}m")
        
        if distance < min_distance:
            self.get_logger().warn(f"Goal is very close to start position ({distance:.2f}m < {min_distance}m)")
            return False
        return True

    def calculate_path_length(self, path):
        """Calculate total path length from waypoint sequence"""
        if len(path) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(1, len(path)):
            dx = path[i]['x'] - path[i-1]['x']
            dy = path[i]['y'] - path[i-1]['y'] 
            dz = path[i]['z'] - path[i-1]['z']
            segment_length = math.sqrt(dx*dx + dy*dy + dz*dz)
            total_length += segment_length
        
        return total_length

    def calculate_straight_line_distance(self, start_point, end_point):
        """Calculate direct distance between two points"""
        dx = end_point['x'] - start_point['x']
        dy = end_point['y'] - start_point['y']
        dz = end_point['z'] - start_point['z']
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    def average_deviation_interpolated(self,planned, actual):
        """For each planned point, find the closest actual pose. Average the distances."""
        if not planned or not actual:
            return None
        deviations = []
        for ref in planned:
            best_dist = float('inf')
            for pose in actual:
                d = math.sqrt(
                    (ref['x'] - pose['x'])**2 +
                    (ref['y'] - pose['y'])**2 +
                    (ref['z'] - pose['z'])**2
                )
                if d < best_dist:
                    best_dist = d
            deviations.append(best_dist)
        return sum(deviations) / len(deviations) if deviations else None

    def calculate_path_efficiency(self, planned_path, actual_path, start_pose, goal_pose):
        """Calculate path efficiency metrics"""
        metrics = {
            'planned_length_m': 0.0,
            'actual_length_m': 0.0,
            'optimal_length_m': 0.0,
            'planned_efficiency': 0.0,
            'actual_efficiency': 0.0,
            'path_deviation_m': 0.0
        }
        
        # Calculate path lengths
        if planned_path:
            metrics['planned_length_m'] = self.calculate_path_length(planned_path)
        
        if actual_path:
            metrics['actual_length_m'] = self.calculate_path_length(actual_path)
        
        # Calculate optimal (straight-line) distance
        start_point = {
            'x': start_pose.pose.position.x,
            'y': start_pose.pose.position.y, 
            'z': start_pose.pose.position.z
        }
        goal_point = {
            'x': goal_pose.pose.position.x,
            'y': goal_pose.pose.position.y,
            'z': goal_pose.pose.position.z
        }
        metrics['optimal_length_m'] = self.calculate_straight_line_distance(start_point, goal_point)
        
        # Calculate efficiency ratios
        if metrics['planned_length_m'] > 0:
            metrics['planned_efficiency'] = metrics['optimal_length_m'] / metrics['planned_length_m']
        
        if metrics['actual_length_m'] > 0:
            metrics['actual_efficiency'] = metrics['optimal_length_m'] / metrics['actual_length_m']
        
        # Calculate average path deviation using existing function
        if planned_path and actual_path:
            metrics['path_deviation_m'] = self.average_deviation_interpolated(planned_path, actual_path) or 0.0
        
        return metrics

    def validate_navigation_success(self, start_pose, goal_pose, status_code, remaining_distance, threshold):
        """
        Enhanced navigation success validation with clear assertions
        
        Validates navigation success with clear assertions:
        1. Nav2 action succeeded (status code 4)
        2. Robot reached goal within distance threshold

        Returns: True if all assertions pass, False otherwise
        """
        # Assertion 1: Navigation action succeeded
        if status_code != 4:  # 4 = SUCCEEDED
            self.get_logger().error(f"❌ Navigation failed: status code {status_code} != 4 (SUCCEEDED)")
            return False
        
        # Assertion 2: Robot reached goal within threshold
        if remaining_distance is not None and remaining_distance >= threshold:
            self.get_logger().error(f"❌ Distance assertion failed: {remaining_distance:.3f}m >= {threshold}m threshold")
            return False
        
        self.get_logger().info(f"✅ Navigation assertions passed: "
                             f"status=SUCCEEDED, remaining_dist={remaining_distance:.3f}m < {threshold}m, "
                             f"planned_waypoints={len(self.planned_path)}, actual_points={len(self.actual_path)}")
        return True

    def run_single_test(self, start_pose, goal_pose, test_name, threshold):
        # Reset plan capture flag for each new test
        self._feedback_data = []
        self.planned_path = []
        self.actual_path = []
        self.initial_plan_captured = False  # Reset flag to capture new plan
        
        # Validate that goal requires movement
        if not self.validate_goal_distance(start_pose, goal_pose):
            self.get_logger().warn("Goal is too close to start - test may appear to succeed instantly")
        
        # Clear any existing state
        self.cancel_all_goals()
        time.sleep(1.0)  # Allow time for cancellation
        
        # Record start time
        self._start_time = self.get_clock().now()
        
        # Create and send goal with fresh timestamp
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()  # Fresh timestamp
        
        self.get_logger().info(f"Sending goal: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})")
        
        # Wait for server and send goal
        if not self.nav_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Navigation action server not available!")
            return None, 0.0
            
        future = self.nav_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected!")
            return None, 0.0
            
        self.get_logger().info("Goal accepted, waiting for result...")
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        status_code = result_future.result().status
        # duration = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        
        # Process feedback
        if self._feedback_data:
            last_feedback = self._feedback_data[-1]
            remaining_distance = last_feedback.distance_remaining
            number_of_recoveries = last_feedback.number_of_recoveries
        else:
            remaining_distance = None
            number_of_recoveries = None
        if hasattr(last_feedback, 'navigation_time'):
            duration = last_feedback.navigation_time.sec + last_feedback.navigation_time.nanosec * 1e-9
        elif hasattr(last_feedback, 'elapsed_time'):
            duration = last_feedback.elapsed_time.sec + last_feedback.elapsed_time.nanosec * 1e-9
        self.get_logger().info(f"Navigation completed: status={self.STATUS_CODE_TO_STRING.get(status_code)}, duration={duration:.2f}s")

        # Validate navigation success
        test_passed = self.validate_navigation_success(
            start_pose, goal_pose, status_code, remaining_distance, threshold)
        
        status = "PASS" if test_passed else "FAIL"
        
        # Log suspicious results
        if duration < 1.0 and status == "PASS":
            self.get_logger().warn(f"Suspiciously fast completion ({duration:.2f}s) - possible instant success bug")
        
        # Calculate path efficiency metrics
        path_metrics = self.calculate_path_efficiency(self.planned_path, self.actual_path, start_pose, goal_pose)
        
        # Log path efficiency metrics
        self.get_logger().info(f"📊 Path Analysis: planned={path_metrics['planned_length_m']:.2f}m, "
                             f"actual={path_metrics['actual_length_m']:.2f}m, "
                             f"optimal={path_metrics['optimal_length_m']:.2f}m, "
                             f"efficiency={path_metrics['actual_efficiency']:.3f}")
            
        deviation = self.average_deviation_interpolated(self.planned_path, self.actual_path) 
    

        run_report = {
            'test_name': test_name,
            'start': {'x': start_pose.pose.position.x, 'y': start_pose.pose.position.y, 'z': start_pose.pose.position.z},
            'goal': {'x': goal_pose.pose.position.x, 'y': goal_pose.pose.position.y, 'z': goal_pose.pose.position.z},
            'test_status': status,
            'nav2_status': self.STATUS_CODE_TO_STRING.get(status_code, 'UNKNOWN'),
            'number_of_recoveries': number_of_recoveries,
            'remaining_distance': remaining_distance,
            'duration': duration,
            'path_metrics': path_metrics,
            'timestamp': datetime.now().isoformat(),
            'deviation': deviation,
            'planned_path': self.planned_path,
            'actual_path': self.actual_path,
        }
        
        fname = self.save_run_report(test_name, run_report)
        run_summary = {
            'test_name': test_name,
            'status': status,
            'duration': duration,
            'detail_file': fname,
        }
        self.per_run_summaries.append(run_summary)
        return run_report, duration

    def save_run_report(self, test_name, report):
        os.makedirs('reports', exist_ok=True)
        fname = f'reports/{test_name}_result.yaml'
        with open(fname, 'w') as f:
            yaml.dump(report, f, sort_keys=False)
        msg = String()
        msg.data = yaml.dump(report)
        self.result_pub.publish(msg)
        return fname

    def save_summary_report(self, test_name, repetitions):
        n_success = sum(1 for r in self.run_results if r['test_status'] == 'PASS')
        successful_durations = [d for i, d in enumerate(self.run_durations) if self.run_results[i]['test_status'] == 'PASS']
        avg_time = sum(successful_durations) / len(successful_durations) if successful_durations else 0.0
        min_time = min(successful_durations) if successful_durations else 0.0
        max_time = max(successful_durations) if successful_durations else 0.0
        success_rate = n_success / repetitions if repetitions > 0 else 0.0
        last_run = self.run_results[-1] if self.run_results else {}

        # Average deviation across all successful runs
        successful_runs_with_paths = [r for r in self.run_results 
                                    if r.get('test_status') == 'PASS' 
                                    and r.get("planned_path") 
                                    and r.get("actual_path")]

        deviations = [run["deviation"] for run in successful_runs_with_paths]

        avg_deviation = sum(deviations) / len(deviations) if len(deviations) > 0 else 0.0

        # Calculate average path metrics across all runs
        avg_metrics = {
            'avg_planned_length': 0.0,
            'avg_actual_length': 0.0,
            'avg_optimal_length': 0.0,
            'avg_planned_efficiency': 0.0,
            'avg_actual_efficiency': 0.0,
            'avg_path_deviation': 0.0
        }
        
        if self.run_results:
            valid_runs = [r for r in self.run_results if r.get('path_metrics')]
            if valid_runs:
                n_valid = len(valid_runs)
                avg_metrics['avg_planned_length'] = sum(r['path_metrics']['planned_length_m'] for r in valid_runs) / n_valid
                avg_metrics['avg_actual_length'] = sum(r['path_metrics']['actual_length_m'] for r in valid_runs) / n_valid
                avg_metrics['avg_optimal_length'] = sum(r['path_metrics']['optimal_length_m'] for r in valid_runs) / n_valid
                avg_metrics['avg_planned_efficiency'] = sum(r['path_metrics']['planned_efficiency'] for r in valid_runs) / n_valid
                avg_metrics['avg_actual_efficiency'] = sum(r['path_metrics']['actual_efficiency'] for r in valid_runs) / n_valid
                avg_metrics['avg_path_deviation'] = sum(r['path_metrics']['path_deviation_m'] for r in valid_runs) / n_valid

        # Include complete reference planned path and actual path from best/last run
        reference_planned_path = []
        reference_actual_path = []
        
        reference_planned_path = []
        reference_actual_path = []

        if self.run_results:
            successful_runs = [r for r in self.run_results if r.get('test_status') == 'PASS']
            if successful_runs:
                # Use the fastest successful run as reference
                reference_run = min(successful_runs, key=lambda x: x.get('duration', float('inf')))
                reference_planned_path = reference_run.get('planned_path', [])
                reference_actual_path = reference_run.get('actual_path', [])
            else:
                # If no successful runs, use the last run (even if failed)
                reference_run = self.run_results[-1]
                reference_planned_path = reference_run.get('planned_path', [])
                reference_actual_path = reference_run.get('actual_path', [])

        # summary with success statistics
        summary = {
            "test_name": test_name,
            "start_pose": {
                "x": self.get_parameter('start_x').value,
                "y": self.get_parameter('start_y').value,
                "z": self.get_parameter('start_z').value,
                "yaw": self.get_parameter('start_yaw').value
            },
            "goal_pose": {
                "x": self.get_parameter('goal_x').value,
                "y": self.get_parameter('goal_y').value,
                "z": self.get_parameter('goal_z').value,        
                "yaw": self.get_parameter('goal_yaw').value
            },
            "test_configuration": {
                "repetitions": repetitions,
                "total_runs": len(self.run_results),
                "successful_runs": n_success,
                "failed_runs": repetitions - n_success,
                "success_rate_percent": round(success_rate * 100, 1)
            },
            "timing_statistics": {
                "average_time_s": round(avg_time, 2),
                "min_time_s": round(min_time, 2),
                "max_time_s": round(max_time, 2),
                "time_std_dev_s": round(self._calculate_std_dev(successful_durations), 2),
                "all_successful_durations_s": [round(d, 2) for d in successful_durations]
            },
            "path_performance": avg_metrics,
            "legacy_metrics": {
                "average_deviation": avg_deviation  # Keep for backward compatibility
            },
            # Include complete reference paths
            "reference_paths": {
                "description": f"Complete paths from {'best successful' if repetitions > 1 and n_success > 0 else 'last'} run",
                "planned_path_full": reference_planned_path,
                "actual_path_full": reference_actual_path,
                "planned_waypoints_count": len(reference_planned_path),
                "actual_points_count": len(reference_actual_path)
            },
            "per_run_details": self.per_run_summaries
        }
        
        # Save both summary and detailed reference paths
        summary_file = f'reports/{test_name}_summary.yaml'
        with open(summary_file, 'w') as f:
            yaml.dump(summary, f, sort_keys=False)
        
        # compact summary without full paths for quick review
        compact_summary = {k: v for k, v in summary.items() if k != 'reference_paths'}
        compact_file = f'reports/{test_name}_summary_compact.yaml'
        with open(compact_file, 'w') as f:
            yaml.dump(compact_summary, f, sort_keys=False)
            
        self.get_logger().info(f"Summary written with complete paths: {summary_file}")
        self.get_logger().info(f"Compact summary (no paths): {compact_file}")
        
        # console logging with success statistics
        self.get_logger().info(f"📊 TEST SUMMARY: {test_name}")
        self.get_logger().info(f"   Success Rate: {n_success}/{repetitions} ({success_rate*100:.1f}%)")
        self.get_logger().info(f"   Timing: avg={avg_time:.1f}s, min={min_time:.1f}s, max={max_time:.1f}s")
        self.get_logger().info(f"   Path Efficiency: avg={avg_metrics['avg_actual_efficiency']:.3f}")
        self.get_logger().info(f"   Path Length: avg_actual={avg_metrics['avg_actual_length']:.2f}m, avg_planned={avg_metrics['avg_planned_length']:.2f}m")
        self.get_logger().info(f"   Reference Paths: {len(reference_planned_path)} planned, {len(reference_actual_path)} actual waypoints")

    def _calculate_std_dev(self, values):
        """Helper function to calculate standard deviation"""
        if len(values) < 2:
            return 0.0
        mean = sum(values) / len(values)
        variance = sum((x - mean) ** 2 for x in values) / (len(values) - 1)
        return math.sqrt(variance)
    
    def run_tests(self):
        """
        runs the navigation tests based on parameters
        - Supports both single test and batch test modes
        - If test_file is provided, runs all tests in the YAML file sequentially
        - Otherwise runs a single test with individual parameters
        """
        
        test_file = self.get_parameter('test_file').value
        
        if test_file:
            # Batch test mode - run multiple tests from YAML file
            if not os.path.exists(test_file):
                self.get_logger().error(f"Test file '{test_file}' does not exist!")
                return
            
            self.get_logger().info(f"Loading batch test configuration from: {test_file}")
            with open(test_file, 'r') as f:
                test_config = yaml.safe_load(f)
            
            # Check if this is a batch test file with 'tests' section
            if 'tests' in test_config:
                self.run_batch_tests(test_config)
            else:
                # Single test from file
                self.run_single_test_from_config(test_config)
        else:
            # Single test mode using individual parameters
            self.get_logger().info("Running single test with individual parameters")
            self.run_single_test_from_parameters()
    
    def run_batch_tests(self, test_config):
        """Run multiple tests from batch configuration"""
        
        # Get global settings
        world_name = test_config.get('world_name', 'depot')
        entity_name = test_config.get('entity_name', 'turtlebot4')
        
        tests = test_config.get('tests', [])
        if not tests:
            self.get_logger().error("No tests found in configuration file!")
            return
        
        self.get_logger().info(f"Found {len(tests)} tests to run")
        
        total_tests = len(tests)
        passed_tests = 0
        failed_tests = 0
        total_test_iters = 0
        sucessful_tests = 0 # number of tests with all repetitions as sucess
        
        for i, test in enumerate(tests, 1):
            test_name = test.get('test_name', f'test_{i}')
            self.get_logger().info(f"=== Running Test {i}/{total_tests}: {test_name} ===")
            
            # Extract test parameters
            start_x = float(test.get('start_x', 0.0))
            start_y = float(test.get('start_y', 0.0))
            start_z = float(test.get('start_z', 0.0))
            start_yaw = float(test.get('start_yaw', 0.0))
            goal_x = float(test.get('goal_x', 1.0))
            goal_y = float(test.get('goal_y', 1.0))
            goal_z = float(test.get('goal_z', 0.0))
            goal_yaw = float(test.get('goal_yaw', 0.0))
            repetitions = int(test.get('repetitions', 1))
            threshold = float(test.get('dist_thres', 0.3))
            
            self.get_logger().info(f"Route: ({start_x:.1f}, {start_y:.1f}) → ({goal_x:.1f}, {goal_y:.1f}) x{repetitions}")
            
            # Run this individual test
            n_success = self.execute_navigation_test(
                start_x, start_y, start_z, start_yaw,
                goal_x, goal_y, goal_z, goal_yaw,
                test_name, threshold, repetitions, world_name, entity_name
            )
            # Update counts
            passed_tests += n_success
            total_test_iters += repetitions

            # Conditional logging based on actual results
            if n_success == repetitions:
                self.get_logger().info(f"✅ Test {i} PASSED: {test_name}")
                sucessful_tests+=1
            else:
                self.get_logger().error(f"❌ Test {i} FAILED: {test_name} ({n_success}/{repetitions} successful)")
            
        
            time.sleep(1)

        # Summary
        failed_tests = total_tests - passed_tests  # Calculate failed this way
        self.get_logger().info(f"=== Batch Test Summary ===")
        self.get_logger().info(f"Total tests: {total_tests}")
        self.get_logger().info(f"Total tests iterations: {total_test_iters}")
        self.get_logger().info(f"Passed: {passed_tests}")
        self.get_logger().info(f"Failed: {failed_tests}")
        self.get_logger().info(f"Success rate: {passed_tests/total_test_iters*100:.1f}%")
        self.get_logger().info(f"Fully passed tests rate: {sucessful_tests}/{total_tests}")
        
                    

    
    def run_single_test_from_config(self, test_config):
        """Run single test from config file"""     
        # Extract parameters from config
        start_x = float(test_config.get('start_x', 0.0))
        start_y = float(test_config.get('start_y', 0.0))
        start_z = float(test_config.get('start_z', 0.0))
        start_yaw = float(test_config.get('start_yaw', 0.0))
        goal_x = float(test_config.get('goal_x', 1.0))
        goal_y = float(test_config.get('goal_y', 1.0))
        goal_z = float(test_config.get('goal_z', 0.0))
        goal_yaw = float(test_config.get('goal_yaw', 0.0))
        test_name = test_config.get('test_name', 'config_test')
        threshold = float(test_config.get('dist_thres', 0.3))
        repetitions = int(test_config.get('repetitions', 1))
        world_name = test_config.get('world_name', 'depot')
        entity_name = test_config.get('entity_name', 'turtlebot4')
        
        self.get_logger().info(f"Running single test from config: {test_name}")
        
        self.execute_navigation_test(
            start_x, start_y, start_z, start_yaw,
            goal_x, goal_y, goal_z, goal_yaw,
            test_name, threshold, repetitions, world_name, entity_name
        )

    def run_single_test_from_parameters(self):
        """Run single test using ROS parameters"""
        
        start_x = self.get_parameter('start_x').value
        start_y = self.get_parameter('start_y').value
        start_z = self.get_parameter('start_z').value
        start_yaw = self.get_parameter('start_yaw').value
        goal_x = self.get_parameter('goal_x').value
        goal_y = self.get_parameter('goal_y').value
        goal_z = self.get_parameter('goal_z').value
        goal_yaw = self.get_parameter('goal_yaw').value
        test_name = self.get_parameter('test_name').value
        threshold = self.get_parameter('dist_thres').value
        repetitions = int(self.get_parameter('repetitions').value)
        world_name = self.get_parameter('world_name').value
        entity_name = self.get_parameter('entity_name').value
        
        self.get_logger().info(f"Running single test with parameters: {test_name}")
        
        self.execute_navigation_test(
            start_x, start_y, start_z, start_yaw,
            goal_x, goal_y, goal_z, goal_yaw,
            test_name, threshold, repetitions, world_name, entity_name
        )
    def _parse_compound_parameters(self):
        """Parse compound start/goal parameters and override individual ones"""
        
        start_param = self.get_parameter('start').value
        goal_param = self.get_parameter('goal').value
        
        if start_param:
            try:
                start_values = [float(x.strip()) for x in start_param.split(',')]
                if len(start_values) >= 2:
                    self.set_parameters([
                        rclpy.Parameter('start_x', rclpy.Parameter.Type.DOUBLE, start_values[0]),
                        rclpy.Parameter('start_y', rclpy.Parameter.Type.DOUBLE, start_values[1])
                    ])
                    if len(start_values) >= 3:
                        self.set_parameters([
                            rclpy.Parameter('start_z', rclpy.Parameter.Type.DOUBLE, start_values[2])
                        ])
                    if len(start_values) >= 4:
                        self.set_parameters([
                            rclpy.Parameter('start_yaw', rclpy.Parameter.Type.DOUBLE, start_values[3])
                        ])
                    self.get_logger().info(f"Parsed start parameter: {start_values}")
                else:
                    self.get_logger().error(f"Invalid start parameter format: {start_param}")
            except ValueError as e:
                self.get_logger().error(f"Failed to parse start parameter '{start_param}': {e}")
        
        if goal_param:
            try:
                goal_values = [float(x.strip()) for x in goal_param.split(',')]
                if len(goal_values) >= 2:
                    self.set_parameters([
                        rclpy.Parameter('goal_x', rclpy.Parameter.Type.DOUBLE, goal_values[0]),
                        rclpy.Parameter('goal_y', rclpy.Parameter.Type.DOUBLE, goal_values[1])
                    ])
                    if len(goal_values) >= 3:
                        self.set_parameters([
                            rclpy.Parameter('goal_z', rclpy.Parameter.Type.DOUBLE, goal_values[2])
                        ])
                    if len(goal_values) >= 4:
                        self.set_parameters([
                            rclpy.Parameter('goal_yaw', rclpy.Parameter.Type.DOUBLE, goal_values[3])
                        ])
                    self.get_logger().info(f"Parsed goal parameter: {goal_values}")
                else:
                    self.get_logger().error(f"Invalid goal parameter format: {goal_param}")
            except ValueError as e:
                self.get_logger().error(f"Failed to parse goal parameter '{goal_param}': {e}")
    def execute_navigation_test(self, start_x, start_y, start_z, start_yaw,
                          goal_x, goal_y, goal_z, goal_yaw,
                          test_name, threshold, repetitions, world_name, entity_name):
        """
        runs the navigation tests based on parameters
        - Teleports robot to start pose
        - Sets initial pose in AMCL
        - Waits for Nav2 to be ready
        - Runs multiple iterations of navigation to goal
        - Collects results and saves detailed reports
        - Validates navigation success and path efficiency
        - Saves summary report with all runs and metrics
        """

        world_name = self.get_world_name() 

        self.get_logger().info(f"🚀 Starting {repetitions} test iteration(s) for {test_name}")
        self.get_logger().info(f"📍 Route: ({start_x:.2f}, {start_y:.2f}) → ({goal_x:.2f}, {goal_y:.2f})")

        for i in range(repetitions):
            self.get_logger().info(f"🚀 Starting test iteration {i+1}/{repetitions} ------")
            
            # Clear any existing goals and costmaps
            self.cancel_all_goals()
            self.clear_costmaps()
            
            # Teleport robot in simulation
            success = self.teleport_robot(world_name, entity_name, start_x, start_y, start_z, start_yaw)
            if not success:
                self.get_logger().error(f"Failed to teleport robot for run {i+1}")
                continue
                
            # Wait a moment for physics to settle
            time.sleep(2.0)
            
            # Set initial pose (AMCL) and wait for update
            self.set_initial_pose(start_x, start_y, start_z, start_yaw)
            
            # Wait for Nav2 to be ready
            self.wait_for_nav2_active()
            
            # Additional settling time
            time.sleep(1.0)
            
            # Create poses with fresh timestamps
            start = make_pose(start_x, start_y, start_yaw, z=start_z)
            goal = make_pose(goal_x, goal_y, goal_yaw, z=goal_z)
            
            # Run the test
            result, duration = self.run_single_test(
                start, goal, test_name=f"{test_name}_run{i+1}", threshold=threshold)
            
            if result is not None:
                self.run_results.append(result)
                self.run_durations.append(duration)
                self.get_logger().info(f"✅ Run {i+1} completed: status={result['test_status']} duration={duration:.2f}s")
            else:
                self.get_logger().error(f"❌ Run {i+1} failed")
                
            # Inter-test delay
            if i < repetitions - 1:  # Don't wait after the last test
                self.get_logger().info(f"Waiting 1 second before next test...")
                time.sleep(1.0)
                
        self.save_summary_report(test_name, repetitions)
        n_success = sum(1 for r in self.run_results if r['test_status'] == 'PASS')
        self.get_logger().info(f"🚀 Test {test_name} completed: {n_success} PASS")
        return n_success 

def make_pose(x, y, yaw, frame_id="map", z=0.0):
    """Pose creation utility function"""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = rclpy.clock.Clock().now().to_msg()  # Always fresh timestamp
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose

def main(args=None):
    rclpy.init(args=args)
    node = Nav2TestNode()
    try:
        node.run_tests()
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()