

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class DrawS(Node):
    def __init__(self):
        super().__init__('draw_s')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.draw_s)
        self.stage = 0

    def draw_s(self):
        twist = Twist()
        
        if self.stage == 0:
            
            twist.linear.x = 0.2
            self.cmd_vel_pub.publish(twist)
            time.sleep(1.5)
            self.stage = 1

        elif self.stage == 1:
            
            twist.linear.x = 0.2
            twist.angular.z = 0.5
            self.cmd_vel_pub.publish(twist)
            time.sleep(2.5)
            self.stage = 2

        elif self.stage == 2:
          
            twist.linear.x = 0.2
            twist.angular.z = -0.5
            self.cmd_vel_pub.publish(twist)
            time.sleep(2.5)
            self.stage = 3

        elif self.stage == 3:
           
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Finished drawing 'S'")
            self.stage = 4
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    draw_s = DrawS()
    rclpy.spin(draw_s)
    draw_s.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


