#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
import math
import numpy as np

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        # Publishers
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_planned', 10)
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)
        
        # Timer
        self.timer = self.create_timer(0.1, self.planning_callback)
        
        self.laser_data = None
        self.get_logger().info('Path Planner Node Started')

    def laser_callback(self, msg):
        self.laser_data = msg

    def planning_callback(self):
        if self.laser_data is None:
            return
            
        # Simple obstacle avoidance logic
        cmd_msg = Twist()
        
        # Get front, left, and right distances
        ranges = np.array(self.laser_data.ranges)
        front_ranges = ranges[350:] + ranges[:10]  # Front 20 degrees
        left_ranges = ranges[80:100]   # Left side
        right_ranges = ranges[260:280]  # Right side
        
        front_dist = np.min(front_ranges[np.isfinite(front_ranges)])
        left_dist = np.mean(left_ranges[np.isfinite(left_ranges)])
        right_dist = np.mean(right_ranges[np.isfinite(right_ranges)])
        
        # Decision making
        if front_dist > 2.5:
            # Clear ahead, move forward
            cmd_msg.linear.x = 0.3
            cmd_msg.angular.z = 0.0
        elif left_dist > right_dist:
            # Turn left
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.5
        else:
            # Turn right
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = -0.5
            
        self.cmd_vel_pub.publish(cmd_msg)
        
        # Create and publish planned path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "base_link"
        
        # Simple path ahead
        for i in range(10):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(i * 0.3)
            pose.pose.position.y = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    path_planner = PathPlanner()
    rclpy.spin(path_planner)
    path_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
