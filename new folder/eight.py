#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pi

class TurtleBot(Node):

    def __init__(self):
        super().__init__('draw_eight')
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.callback, 10)
        self.pose = Pose()
        self.rate = self.create_rate(10)

    def callback(self, data):
        self.pose = data

    def move_circle(self, radius, linear_speed, angular_speed, duration):
        vel_msg = Twist()
        vel_msg.linear.x = linear_speed
        vel_msg.angular.z = angular_speed

        start_time = self.get_clock().now()

        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def draw_eight(self):
        radius = 1.0
        linear_speed = 1.0
        angular_speed = linear_speed / radius

        
        circle_duration = 2 * pi / angular_speed

    
        self.move_circle(radius, linear_speed, angular_speed, circle_duration / 2)

        
        self.move_circle(0, linear_speed, 0, 1.0)

        
        self.move_circle(radius, linear_speed, -angular_speed, circle_duration / 2)

        
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    turtlebot = TurtleBot()
    turtlebot.draw_eight()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
