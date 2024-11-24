#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetKinematicsPose
from open_manipulator_msgs.msg import KinematicsPose
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import String
from threading import Event
import re
import json
from datetime import datetime

class MoveUpwardClient(Node):
    def __init__(self):
        super().__init__('move_upward_client')
        
        # Declare parameters (target coordinates and constants)
        self.declare_parameter('target_x', 294.0)  # Example value, modify as needed
        self.declare_parameter('target_y', 416.0)  # Example value, modify as needed
        self.declare_parameter('kx', -0.0003328895)
        self.declare_parameter('ky', -0.0003328895)
        self.declare_parameter('fixed_z', 0.6)  # Fixed Z-axis value
        
        # Read parameter values
        self.target_x = self.get_parameter('target_x').get_parameter_value().double_value
        self.target_y = self.get_parameter('target_y').get_parameter_value().double_value
        self.kx = self.get_parameter('kx').get_parameter_value().double_value
        self.ky = self.get_parameter('ky').get_parameter_value().double_value
        self.fixed_z = self.get_parameter('fixed_z').get_parameter_value().double_value

        self.get_logger().info(f"Parameters loaded: target_x={self.target_x}, target_y={self.target_y}, kx={self.kx}, ky={self.ky}, fixed_z={self.fixed_z}")

        # Define fixed orientation
        self.fixed_orientation = Quaternion()
        self.fixed_orientation.x = 0.0
        self.fixed_orientation.y = 0.7189371190367781
        self.fixed_orientation.z = 0.0
        self.fixed_orientation.w = 0.6950751174305533

        self.get_logger().info(f"Fixed orientation set to: {self.fixed_orientation}")

        # Create service client
        self.cli = self.create_client(SetKinematicsPose, 'goal_task_space_path')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = SetKinematicsPose.Request()

        # Current pose storage
        self.current_pose = Pose()
        self.pose_received = Event()

        # Subscriber for current kinematics pose
        self.subscription_pose = self.create_subscription(
            KinematicsPose,
            'kinematics_pose',
            self.kinematics_pose_callback,
            10)
        self.subscription_pose  # prevent unused variable warning

        # Subscriber for pixel coordinates
        self.subscription_pixel = self.create_subscription(
            String,
            '/pixel_coord',
            self.pixel_coord_callback,
            10)
        self.subscription_pixel  # prevent unused variable warning

        self.get_logger().info('MoveUpwardClient node has been started.')

    def kinematics_pose_callback(self, msg):
        """Update the current pose of the manipulator."""
        self.current_pose = msg.pose
        self.pose_received.set()
        self.get_logger().debug(f'Current pose received: x={self.current_pose.position.x}, y={self.current_pose.position.y}, z={self.current_pose.position.z}')

    def pixel_coord_callback(self, msg):
        """/pixel_coord topic callback to receive pixel coordinates and update manipulator position."""
        # Extract p_x and p_y from the message
        p_x, p_y = self.parse_pixel_coord(msg.data)
        if p_x is None or p_y is None:
            self.get_logger().error(f'Failed to parse pixel coordinates: {msg.data}')
            return

        self.get_logger().info(f'Pixel coordinates received: p_x={p_x}, p_y={p_y}')

        # Calculate p_dx and p_dy
        p_dx = p_x - self.target_x
        p_dy = p_y - self.target_y
        self.get_logger().debug(f'p_dx={p_dx}, p_dy={p_dy}')

        # Calculate dx and dy
        dx = self.kx * p_dx
        dy = self.ky * p_dy
        self.get_logger().debug(f'dx={dx}, dy={dy}')

        # Ensure current pose has been received
        if not self.pose_received.is_set():
            self.get_logger().info('Waiting for current pose...')
            self.pose_received.wait(timeout=5.0)
            if not self.pose_received.is_set():
                self.get_logger().error('Timeout: Failed to receive current pose.')
                return

        # Set new target pose
        self.req.end_effector_name = 'gripper'  # Update with your manipulator's end effector name if different
        self.req.kinematics_pose.pose.position.x = self.current_pose.position.x + dy
        self.req.kinematics_pose.pose.position.y = self.current_pose.position.y + dx
        self.req.kinematics_pose.pose.position.z = self.fixed_z  # Fixed Z-axis
        self.req.kinematics_pose.pose.orientation = self.fixed_orientation  # Fixed orientation
        self.req.path_time = 1.0  # Path movement time (adjust as needed)

        self.get_logger().info(f'Target pose set: x={self.req.kinematics_pose.pose.position.x:.4f}, y={self.req.kinematics_pose.pose.position.y:.4f}, z={self.req.kinematics_pose.pose.position.z:.4f}')
        self.get_logger().info(f'Fixed orientation applied: {self.req.kinematics_pose.pose.orientation}')

        # Send service request asynchronously
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.service_response_callback)

    def parse_pixel_coord(self, data):
        """
        Extract p_x and p_y from string format '(300.0, 420.0)'.
        Returns (p_x, p_y) or (None, None) on failure.
        """
        pattern = r'\(\s*([0-9]*\.?[0-9]+)\s*,\s*([0-9]*\.?[0-9]+)\s*\)'
        match = re.match(pattern, data)
        if match:
            try:
                p_x = float(match.group(1))
                p_y = float(match.group(2))
                return p_x, p_y
            except ValueError:
                return None, None
        else:
            return None, None

    def service_response_callback(self, future):
        """Handle the service response."""
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        else:
            if response.is_planned:
                self.get_logger().info('Manipulator moved successfully.')
            else:
                self.get_logger().error('Failed to move manipulator.')

    def destroy_node(self):
        """Clean up resources when the node is destroyed."""
        self.get_logger().info('Shutting down MoveUpwardClient node.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    client = MoveUpwardClient()

    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
