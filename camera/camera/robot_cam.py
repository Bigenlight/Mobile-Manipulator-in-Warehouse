#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RobotCam(Node):
    def __init__(self):
        super().__init__('robot_cam')
        self.publisher_ = self.create_publisher(Image, '/image_raw/compressed', 10)
        self.bridge = CvBridge()
        self.get_logger().info("Initializing RobotCam Node")

        # Attempt to open the camera
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Explicitly specify V4L2 backend
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera at index 0. Trying index 1...")
            self.cap = cv2.VideoCapture(1, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.get_logger().fatal("Cannot open camera at index 1. Shutting down node.")
                rclpy.shutdown()
                return
            else:
                self.get_logger().info("Camera successfully opened at index 1.")
        else:
            self.get_logger().info("Camera successfully opened at index 0.")

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Failed to capture image')
            return

        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f'CvBridge conversion error: {e}')
            return

        self.publisher_.publish(ros_image)
        self.get_logger().debug('Published an image')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    robot_cam = RobotCam()
    try:
        rclpy.spin(robot_cam)
    except KeyboardInterrupt:
        pass
    robot_cam.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
