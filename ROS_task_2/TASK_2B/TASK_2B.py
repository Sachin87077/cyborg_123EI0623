import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
from tf_transformations import euler_from_quaternion 

class Task2Bcontroller(Node):
    def __init__(self):
        super().__init__('Task2Bcontroller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription 

        
        self.current_position = None
        self.current_orientation = None

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        self.current_orientation = self.euler_from_quaternion(orientation_q)
        
        # circular moiton
        self.move_in_circle()

    def euler_from_quaternion(self, quaternion):
        """
       
        """
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return roll_x, pitch_y, yaw_z

    def move_in_circle(self):
        twist = Twist()
        twist.linear.x = 10.0
        twist.angular.z = 10.0  
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    movement_control = Task2Bcontroller()
    rclpy.spin(movement_control)

    movement_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

