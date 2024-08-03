import rclpy
import cv2
from cv2 import aruco
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from sensor_msgs.msg import Image


class Feedback(Node):
    def __init__(self):
        super().__init__('feed')

        self.image_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pose_publisher = self.create_publisher(Pose2D, '/bot_pose', 10)

        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.desired_marker_id = 1

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.adaptiveThreshold(gray, 250, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, blockSize=20, C=2)
        detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)  
        corners, ids, rejected = detector.detectMarkers(gray)
        if ids is not None:
            for i, marker_id in enumerate(ids):
                if marker_id == self.desired_marker_id:
                    self.get_logger().info(" ArUco marker with ID 1 has been DETECTED")
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    marker_center = np.mean(corners[i][0], axis=0)
                    cv2.circle(frame, tuple(marker_center.astype(int)), 5, (0, 255, 0), -1)
                    pose_msg=Pose2D()
                    self.get_logger().info(f"Pose (x,y): {pose_msg.x},{pose_msg.y},theta:{pose_msg.theta}")

        else:
            self.get_logger().info("MARKER NOT DETECTED")
        cv2.imshow(' AURCO MARKER DETECTED  ', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = Feedback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
