#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

class CCTVNode(Node):
    def __init__(self):
        super().__init__('cctv_node')
        self.publisher_ = self.create_publisher(Image, '/cctv_imj', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.publish_image)  # Publish at 10 Hz

        # Initialize camera
        self.cap = cv.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera 0')
            rclpy.shutdown()
            return
        else:
            self.get_logger().info('Camera 0 opened successfully')

        # Load calibration parameters
        calibration_matrix_path = '/home/theo/3_ws/src/calibration_matrix_webcam2.npy'
        distortion_coefficients_path = '/home/theo/3_ws/src/distortion_coefficients_webcam2.npy'
        self.mtx, self.dist = self.load_calibration_parameters(calibration_matrix_path, distortion_coefficients_path)

        # Precompute undistortion maps
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to read from camera during initialization')
            rclpy.shutdown()
            return

        h, w = frame.shape[:2]
        self.new_mtx, self.roi = cv.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
        self.mapx, self.mapy = cv.initUndistortRectifyMap(self.mtx, self.dist, None, self.new_mtx, (w, h), cv.CV_32FC1)

        self.get_logger().info('Undistortion maps computed')

    def load_calibration_parameters(self, calibration_matrix_path, distortion_coefficients_path):
        try:
            mtx = np.load(calibration_matrix_path)
            dist = np.load(distortion_coefficients_path)
            self.get_logger().info('Calibration parameters loaded successfully.')
            return mtx, dist
        except Exception as e:
            self.get_logger().error(f'Error loading calibration parameters: {e}')
            rclpy.shutdown()

    def publish_image(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture image from camera')
            return

        # Undistort the frame
        undistorted_frame = cv.remap(frame, self.mapx, self.mapy, interpolation=cv.INTER_LINEAR)

        # Convert to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(undistorted_frame, encoding='bgr8')

        # Publish the image
        self.publisher_.publish(msg)
        self.get_logger().info('Published undistorted image')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CCTVNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
