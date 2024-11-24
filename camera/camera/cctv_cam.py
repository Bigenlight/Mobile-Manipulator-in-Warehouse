#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CctvCam(Node):
    def __init__(self):
        super().__init__('cctv_cam')
        self.publisher_ = self.create_publisher(Image, '/image_raw/compressed2', 10)
        self.bridge = CvBridge()
        self.get_logger().info("Initializing CCTVCam Node")

        # Attempt to open the camera (start from index 1 to avoid conflict with robot_cam)
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera at index 1. Trying index 2...")
            self.cap = cv2.VideoCapture(1, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.get_logger().error("Cannot open camera at index 2. Trying index 3...")
                self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
                if not self.cap.isOpened():
                    self.get_logger().fatal("Cannot open camera at any index. Shutting down node.")
                    rclpy.shutdown()
                    return
                else:
                    self.get_logger().info("Camera successfully opened at index 3.")
            else:
                self.get_logger().info("Camera successfully opened at index 2.")
        else:
            self.get_logger().info("Camera successfully opened at index 1.")

        self.timer = self.create_timer(0.2, self.timer_callback)  # 10 Hz

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
    cctv_cam = CctvCam()
    try:
        rclpy.spin(cctv_cam)
    except KeyboardInterrupt:
        pass
    cctv_cam.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
