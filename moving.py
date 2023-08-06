#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleMove(Node):
    def _init_(self):
        super()._init_('turtle_move_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.rate = self.create_rate(10)  # 10 Hz (adjust as needed)
        self.current_pose = Pose()
        self.target_x = 0.0
        self.target_y = 0.0
        self.is_reached = True

    def pose_callback(self, pose):
        self.current_pose = pose

    def calculate_distance(self, x, y):
        return math.sqrt((x - self.current_pose.x) * 2 + (y - self.current_pose.y) * 2)

    def move_to_target(self):
        while rclpy.ok():
            if self.is_reached:
                self.get_new_target()  # Get new target coordinates if the previous target is reached
                self.is_reached = False

            distance = self.calculate_distance(self.target_x, self.target_y)
            if distance < 0.1:  # Set an appropriate threshold for reaching the target
                self.get_logger().info(f"Target ({self.target_x}, {self.target_y}) reached!")
                self.is_reached = True
            else:
                # Calculate the required linear and angular velocities
                linear_vel = 0.5 * distance
                angle_to_target = math.atan2(self.target_y - self.current_pose.y, self.target_x - self.current_pose.x)
                angular_vel = 4 * (angle_to_target - self.current_pose.theta)

                # Limit the angular velocity to a reasonable range (-2 to 2 radians/s)
                angular_vel = max(min(angular_vel, 2.0), -2.0)

                # Create and publish the Twist message
                twist_msg = Twist()
                twist_msg.linear.x = linear_vel
                twist_msg.angular.z = angular_vel
                self.cmd_vel_pub.publish(twist_msg)

            self.rate.sleep()

    def get_new_target(self):
        # Implement a method to set new target coordinates here
        # For example, you can get the new coordinates from user input or any other source
        self.target_x = float(input("Enter new target x-coordinate: "))
        self.target_y = float(input("Enter new target y-coordinate: "))

def main(args=None):
    rclpy.init(args=args)
    turtle_move = TurtleMove()
    turtle_move.get_logger().info("Moving the turtle to the target coordinate...")
    turtle_move.move_to_target()
    rclpy.shutdown()

if _name_ == '_main_':
    main()