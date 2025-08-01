import subprocess
import time
import rclpy

class NavigationTestErrorHandler:
    """Handles error detection, troubleshooting, and cleanup for navigation tests"""
    
    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()
    
    def wait_for_robot_ready(self, timeout=45):
        """Wait for robot controllers and TF frames to be ready"""
        self.logger.info("Waiting for robot initialization...")
        
        start_time = time.time()
        controllers_ready = False
        tf_ready = False
        
        while time.time() - start_time < timeout:
            # Check controllers
            if not controllers_ready:
                try:
                    result = subprocess.run(
                        ["ros2", "control", "list_controllers"], 
                        capture_output=True, text=True, timeout=5
                    )
                    if "joint_state_broadcaster" in result.stdout and "active" in result.stdout:
                        controllers_ready = True
                        self.logger.info("✓ Robot controllers active")
                except:
                    pass
            
            # Check TF frames
            if controllers_ready and not tf_ready:
                try:
                    result = subprocess.run(
                        ["ros2", "run", "tf2_ros", "tf2_echo", "base_link", "base_footprint"], 
                        capture_output=True, text=True, timeout=2
                    )
                    if "Translation" in result.stdout:
                        tf_ready = True
                        self.logger.info("✓ Robot TF frames available")
                except:
                    pass
            
            if controllers_ready and tf_ready:
                self.logger.info("✓ Robot fully initialized")
                return True
                
            time.sleep(2.0)
        
        self.logger.error(f"Robot initialization timed out after {timeout}s")
        return False
    
    def display_setup_failure_message(self, error_type, error_details=None):
        """Display generic setup failure message, stop node, and exit"""
        self.logger.error("=" * 80)
        self.logger.error("NAVIGATION TEST SETUP FAILED")
        self.logger.error("=" * 80)
        
        # Display specific error based on type
        if error_type == "TIMEOUT":
            self.logger.error("Operation timed out - System components may not be ready")
        elif error_type == "PROCESS_ERROR":
            self.logger.error("Process execution failed - System components may be missing")
        elif error_type == "AMCL_NOT_READY":
            self.logger.error("AMCL localization not ready - Pose initialization failed")
        elif error_type == "NAV2_NOT_READY":
            self.logger.error("Nav2 navigation stack not ready - Navigation services unavailable")
        elif error_type == "GAZEBO_NOT_READY":
            self.logger.error("Gazebo simulation not ready - Robot teleportation failed")
        elif error_type == "RVIZ_NOT_READY":
            self.logger.error("RViz visualization not ready - Display components failed")
        elif error_type == "ROBOT_NOT_READY":
            self.logger.error("Robot controllers/TF frames not ready - Robot initialization failed")
        else:
            self.logger.error(f"Unknown error: {error_type}")
        
        # Display error details if provided
        if error_details:
            self.logger.error(f"Details: {error_details}")
        
        self.logger.error("")
        self.logger.error("TROUBLESHOOTING STEPS:")
        self.logger.error("1. Verify Gazebo is fully loaded:")
        self.logger.error("   - Check that Gazebo GUI is open and responsive")
        self.logger.error("   - Ensure TurtleBot4 robot is visible in the simulation")
        self.logger.error("   - If stuck on 'Requesting list of worlds', check internet connectivity:")
        self.logger.error("     * Test: ping gazebosim.org")
        self.logger.error("     * Gazebo downloads world assets on first launch")
        self.logger.error("     * This can take 1-3 minutes depending on connection speed")
        self.logger.error("")
        self.logger.error("2. Verify RViz is properly loaded:")
        self.logger.error("   - Check that RViz window is open")
        self.logger.error("   - Verify the map is displayed (gray/black grid)")
        self.logger.error("   - Confirm robot model appears in RViz")
        self.logger.error("   - If robot/costmaps not visible in RViz:")
        self.logger.error("     * Robot controllers may have failed to initialize")
        self.logger.error("     * Check: ros2 control list_controllers")
        self.logger.error("     * Look for controller_manager/spawner errors in logs")
        self.logger.error("     * TF tree incomplete - robot frames missing")
        self.logger.error("     * Test: ros2 run tf2_tools view_frames.py")
        self.logger.error("   - If seeing TF_OLD_DATA warnings for wheel frames:")
        self.logger.error("     * Multiple RViz instances may be running")
        self.logger.error("     * Check: ps aux | grep rviz")
        self.logger.error("     * Kill old instances: pkill -f rviz")
        self.logger.error("     * This indicates AMCL pose not properly initialized")
        self.logger.error("")
        self.logger.error("3. Check ROS nodes are running:")
        self.logger.error("   Run: ros2 node list | grep -E '(amcl|gazebo)'")
        self.logger.error("   - If seeing RTPS_TRANSPORT_SHM errors:")
        self.logger.error("     * Shared memory transport issues detected")
        self.logger.error("     * Check for leftover ROS processes: ps aux | grep ros")
        self.logger.error("     * Clean up: pkill -f ros2 && pkill -f gazebo && pkill -f rviz")
        self.logger.error("     * In Docker: restart container to reset shared memory")
        self.logger.error("")
        self.logger.error("4. Verify robot controllers and topics:")
        self.logger.error("   Run: ros2 control list_controllers")
        self.logger.error("   Run: ros2 topic list | grep -E '(initialpose|amcl_pose)'")
        self.logger.error("   - If joint_state_broadcaster not active:")
        self.logger.error("     * Robot controllers failed to spawn")
        self.logger.error("     * Check controller_manager logs for spawner errors")
        self.logger.error("     * May need longer initialization time")
        self.logger.error("")
        self.logger.error("5. If issues persist:")
        self.logger.error("   - Close all RViz and Gazebo instances: pkill -f rviz && pkill -f gazebo")
        self.logger.error("   - Restart the container: docker stop ros2_nav2_container && ./docker/run_docker.sh")
        self.logger.error("   - Wait 1-2 minutes for full initialization before running tests")
        self.logger.error("   - For persistent connectivity issues, check firewall/proxy settings")
        self.logger.error("")
        self.logger.error("=" * 80)
        self.logger.error("EXITING - Please resolve the above issues and try again")
        self.logger.error("=" * 80)
        
        # Stop the node and cleanup
        self._cleanup_node()
        
        # Exit the node gracefully
        raise SystemExit(f"Navigation test setup failed: {error_type}")
    
    def _cleanup_node(self):
        """Clean up node resources before exit"""
        self.logger.info("Stopping Nav2TestNode...")
        try:
            # Cancel any active navigation goals
            if hasattr(self.node, 'cancel_all_goals'):
                self.node.cancel_all_goals()
            
            # Destroy action clients and subscriptions
            if hasattr(self.node, 'nav_action_client'):
                self.node.nav_action_client.destroy()
            if hasattr(self.node, 'plan_sub'):
                self.node.destroy_subscription(self.node.plan_sub)
            if hasattr(self.node, 'odom_sub'):
                self.node.destroy_subscription(self.node.odom_sub)
            if hasattr(self.node, 'result_pub'):
                self.node.destroy_publisher(self.node.result_pub)
                
            self.logger.info("Node cleanup completed")
        except Exception as cleanup_error:
            self.logger.warn(f"Node cleanup warning: {cleanup_error}")
