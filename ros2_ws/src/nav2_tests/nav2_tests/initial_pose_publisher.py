#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import sys
import time

def poses_are_close(pose1, pose2, pos_thresh=0.05, yaw_thresh=0.15):
    dx = abs(pose1.position.x - pose2.position.x)
    dy = abs(pose1.position.y - pose2.position.y)
    dz = abs(pose1.position.z - pose2.position.z)
    dyaw = abs(yaw_from_quat(pose1.orientation) - yaw_from_quat(pose2.orientation))
    return (dx < pos_thresh) and (dy < pos_thresh) and (dz < pos_thresh) and (dyaw < yaw_thresh)

def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 1)

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('cov_thresh', 0.5)  # Covariance threshold for acceptance

        self.max_attempts = 50
        self.attempt = 0
        self.pose_accepted = False
        self.timed_out = False

        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        z = self.get_parameter('z').value
        yaw = self.get_parameter('yaw').value

        self.goal_pose = PoseWithCovarianceStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.pose.pose.position.x = x
        self.goal_pose.pose.pose.position.y = y
        self.goal_pose.pose.pose.position.z = z
        self.goal_pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        self.goal_pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pose.pose.covariance[0] = 0.01
        self.goal_pose.pose.covariance[7] = 0.01
        self.goal_pose.pose.covariance[35] = 0.01

        # Ensure /amcl_pose is being published
        self.amcl_pose_received = False

        # Wait for publisher to connect to subscribers
        self.get_logger().info("Waiting for subscribers to /initialpose...")
        while self.count_subscribers('/initialpose') == 0:
            time.sleep(0.2)
        self.get_logger().info("Subscriber detected. Starting initial pose publishing.")

        self.timer = self.create_timer(1.0, self.publish_initial_pose)

    def publish_initial_pose(self):
        if self.pose_accepted or self.timed_out:
            self.timer.cancel()
            return  # Will be handled in main after spin exits

        if self.attempt >= self.max_attempts:
            self.get_logger().warn("TIMEOUT: Timed out waiting for AMCL to accept initial pose.")
            self.timed_out = True
            self.timer.cancel()
            return

        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.goal_pose)
        self.attempt += 1
        self.get_logger().info(f"Published initial pose ({self.attempt}/{self.max_attempts})")

    def amcl_pose_callback(self, msg):
        cov_trace = msg.pose.covariance[0] + msg.pose.covariance[7] + msg.pose.covariance[14]
        if poses_are_close(msg.pose.pose, self.goal_pose.pose.pose) and cov_trace < self.get_parameter('cov_thresh').value:
            self.get_logger().info("POSE_ACCEPTED: Initial pose accepted by AMCL.")
            self.pose_accepted = True
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.pose_accepted:
                node.get_logger().info("Exiting after successful pose set.")
                break
            if node.timed_out:
                node.get_logger().error("Exiting due to initial pose TIMEOUT.")
                sys.exit(1)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0 if node.pose_accepted else 1)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user. Shutting down.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

if __name__ == "__main__":
    main()
