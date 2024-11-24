#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        # Publishers and Subscribers
        self.subscription = self.create_subscription(
            Image,
            '/image_raw/compressed2',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Image, '/image_aruco', 10)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Load camera calibration parameters
        self.camera_matrix = np.array([
            [4.20206927e+02, 0.00000000e+00, 3.09996813e+02],
            [0.00000000e+00, 4.22554304e+02, 2.32743975e+02],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        ])

        self.dist_coeffs = np.array([5.96212387e-02, -1.55974578e-01, -9.76032365e-03, 2.33071430e-03, 1.74747614e-01])

        # Marker length in meters
        self.marker_length = 0.107  # 10.7cm

        # ArUco dictionary and parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        self.get_logger().info('ArUco Node has been started.')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Process the image
            processed_image = self.process_image(cv_image)

            # Convert OpenCV image back to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
            ros_image.header = msg.header  # Preserve the original header

            # Publish the processed image
            self.publisher_.publish(ros_image)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def process_image(self, img):
        # Grayscale conversion
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Marker detection
        corners, ids, rejectedCandidates = self.detector.detectMarkers(gray)

        # If markers are detected
        if ids is not None:
            # Pose estimation for markers
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

            # Create a mapping from marker IDs to indices
            marker_indices = {id[0]: idx for idx, id in enumerate(ids)}

            # Check if reference marker ID41 is detected
            if 41 in marker_indices:
                ref_idx = marker_indices[41]
                rvec_ref = rvecs[ref_idx]
                tvec_ref = tvecs[ref_idx]

                # Compute transformation matrix for the reference marker
                R_ref, _ = cv2.Rodrigues(rvec_ref)
                T_ref_to_cam = np.eye(4)
                T_ref_to_cam[:3, :3] = R_ref
                T_ref_to_cam[:3, 3] = tvec_ref.reshape(3)

                # Compute inverse transformation
                T_cam_to_ref = np.linalg.inv(T_ref_to_cam)

                # Iterate over other markers
                for target_id in [40, 42]:
                    if target_id in marker_indices:
                        target_idx = marker_indices[target_id]
                        rvec_target = rvecs[target_idx]
                        tvec_target = tvecs[target_idx]

                        # Compute transformation matrix for the target marker
                        R_target, _ = cv2.Rodrigues(rvec_target)
                        T_target_to_cam = np.eye(4)
                        T_target_to_cam[:3, :3] = R_target
                        T_target_to_cam[:3, 3] = tvec_target.reshape(3)

                        # Compute relative transformation
                        T_target_to_ref = T_cam_to_ref @ T_target_to_cam

                        # Extract relative position
                        t_target_to_ref = T_target_to_ref[:3, 3]

                        # Compute distance and angle
                        distance = np.linalg.norm(t_target_to_ref)
                        angle = np.degrees(np.arctan2(t_target_to_ref[1], t_target_to_ref[0]))

                        # Calculate text position
                        corner = corners[target_idx][0][0]
                        text_position = (int(corner[0]), int(corner[1]) - 30)

                        # Display relative coordinates, distance, and angle
                        coord_text = f"Rel X: {t_target_to_ref[0]:.2f}m Y: {t_target_to_ref[1]:.2f}m Z: {t_target_to_ref[2]:.2f}m"
                        distance_text = f"Distance to ID41: {distance:.2f}m"
                        angle_text = f"Angle to ID41: {angle:.2f} deg"

                        cv2.putText(img, coord_text, text_position,
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        cv2.putText(img, distance_text, (text_position[0], text_position[1] + 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        cv2.putText(img, angle_text, (text_position[0], text_position[1] + 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                # Draw axes and display marker info
                for i in range(len(ids)):
                    cv2.drawFrameAxes(img, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], self.marker_length * 0.5)

                    # Display marker ID
                    corner = corners[i][0][0]
                    cv2.putText(img, f"ID: {ids[i][0]}", (int(corner[0]), int(corner[1]) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Compute distance from camera
                    tvec = tvecs[i][0]
                    distance = np.linalg.norm(tvec)

                    # Display coordinates and distance
                    coord_text = f"X: {tvec[0]:.2f}m Y: {tvec[1]:.2f}m Z: {tvec[2]:.2f}m"
                    distance_text = f"Distance: {distance:.2f}m"

                    cv2.putText(img, coord_text, (int(corner[0]), int(corner[1]) + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    cv2.putText(img, distance_text, (int(corner[0]), int(corner[1]) + 35),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            else:
                # If reference marker is not detected, process markers as usual
                for i in range(len(ids)):
                    cv2.drawFrameAxes(img, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], self.marker_length * 0.5)

                    # Display marker ID
                    corner = corners[i][0][0]
                    cv2.putText(img, f"ID: {ids[i][0]}", (int(corner[0]), int(corner[1]) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Compute distance from camera
                    tvec = tvecs[i][0]
                    distance = np.linalg.norm(tvec)

                    # Display coordinates and distance
                    coord_text = f"X: {tvec[0]:.2f}m Y: {tvec[1]:.2f}m Z: {tvec[2]:.2f}m"
                    distance_text = f"Distance: {distance:.2f}m"

                    cv2.putText(img, coord_text, (int(corner[0]), int(corner[1]) + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    cv2.putText(img, distance_text, (int(corner[0]), int(corner[1]) + 35),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        return img

def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
