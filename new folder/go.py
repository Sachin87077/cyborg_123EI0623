#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleGoalNav(Node):
    def __init__(self):
        super().__init__('turtle_goal_nav')

        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        self.goals = [[1.5, 8.5], [8.5, 8.5], [5.197, 5.197], [1.5, 1.5], [8.5, 1.5], [5.197, 5.197]]
        self.current_goal_index = 0
        self.current_pose = None
        
        self.timer = self.create_timer(0.1, self.update)

        self.Kp_linear = 1.0
        self.Kp_angular = 4.0

    def pose_callback(self, msg):
        self.current_pose = msg

    def update(self):
        if self.current_pose is None:
            return
        
        if self.current_goal_index < len(self.goals):
            goal = self.goals[self.current_goal_index]
            distance, angle = self.get_distance_and_angle(goal)

            if distance < 0.1:
                self.current_goal_index += 1
                return

            msg = Twist()
            msg.linear.x = self.Kp_linear * distance
            msg.angular.z = self.Kp_angular * angle
            self.publisher.publish(msg)

    def get_distance_and_angle(self, goal):
        delta_x = goal[0] - self.current_pose.x
        delta_y = goal[1] - self.current_pose.y
        distance = math.sqrt(delta_x**2 + delta_y**2)
        angle_to_goal = math.atan2(delta_y, delta_x)
        angle_difference = self.normalize_angle(angle_to_goal - self.current_pose.theta)
        return distance, angle_difference

    def normalize_angle(self, angle):
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = TurtleGoalNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

