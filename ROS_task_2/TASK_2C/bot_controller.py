

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tf_transformations import euler_from_quaternion
import math

class SpiralBot(Node):
    def __init__(self):
        super().__init__('spiral_bot')
        self.pose_sub = self.create_subscription(PoseStamped, '/bot_pose', self.pose_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_pose = None
        self.timer = self.create_timer(0.1, self.move_spiral)

        
        self.a = 0.1
        self.b = 0.01
        self.max_radius = 2.0

        
        self.goal_x = 10.0
        self.goal_y = 10.0
        self.goal_tolerance = 0.1

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def distance_to_goal(self, x, y):
        return math.sqrt((x - self.current_pose.position.x)**2 + (y - self.current_pose.position.y)**2)

    def move_spiral(self):
        if self.current_pose is None:
            return

        twist = Twist()
        angle = 0.0  

        
        radius = self.a + self.b * angle

        if radius > self.max_radius:
            self.get_logger().info("Reached maximum radius")
            self.destroy_node()

        
        if self.distance_to_goal(self.goal_x, self.goal_y) > self.goal_tolerance:
            
            dx = self.goal_x - self.current_pose.position.x
            dy = self.goal_y - self.current_pose.position.y
            goal_angle = math.atan2(dy, dx)

          
             current_yaw = euler_from_quaternion([
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            ])

          
            angular_error = goal_angle - current_yaw
            linear_speed = min(radius, 0.5)  
            angular_speed = 2.0 * angular_error  
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed

            
            self.cmd_vel_pub.publish(twist)

            self.get_logger().info(f"Moving towards goal: x={self.goal_x}, y={self.goal_y}")

        else:
            
            twist.linear.x = 0
            twist.angular.z = 0
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Reached the goal")
            self.destroy_node()

        
        angle += 0.1

def main(args=None):
    rclpy.init(args=args)
    spiral_bot = SpiralBot()
    rclpy.spin(spiral_bot)
    spiral_bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

