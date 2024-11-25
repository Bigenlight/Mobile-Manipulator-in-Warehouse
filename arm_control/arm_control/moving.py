#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetKinematicsPose
from open_manipulator_msgs.msg import KinematicsPose
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import String, Empty
from threading import Event, Timer
import re

class MoveUpwardClient(Node):
    def __init__(self):
        super().__init__('move_upward_client')
        
        # Declare parameters (target coordinates and constants)
        self.declare_parameter('target_x', 294.0)  # Example value, modify as needed
        self.declare_parameter('target_y', 425.0)  # Example value, modify as needed
        self.declare_parameter('kx', -0.0003328895)
        self.declare_parameter('ky', -0.0003328895)
        self.declare_parameter('fixed_z', 0.06)  # Fixed Z-axis value
        
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

        # Publisher to 'target_box' to send 'done'
        self.target_publisher = self.create_publisher(String, 'target_box', 10)

        # Publisher to 'catch' topic
        self.catch_publisher = self.create_publisher(Empty, 'catch', 10)

        # Timer for 'catch' topic and restart
        self.timer = None

        # Tracking flags
        self.tracking_active = True  # Initially tracking is active
        self.done = False  # Indicates whether the target is reached

        self.get_logger().info('MoveUpwardClient node has been started.')

    def kinematics_pose_callback(self, msg):
        """Update the current pose of the manipulator."""
        self.current_pose = msg.pose
        self.pose_received.set()
        self.get_logger().debug(f'Current pose received: x={self.current_pose.position.x}, y={self.current_pose.position.y}, z={self.current_pose.position.z}')

    def pixel_coord_callback(self, msg):
        """/pixel_coord topic callback to receive pixel coordinates and update manipulator position."""
        if not self.tracking_active:
            # Tracking is inactive, ignore further processing
            return

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

        # Calculate distance to the target point
        distance = (p_dx ** 2 + p_dy ** 2) ** 0.5
        self.get_logger().info(f'Distance to target: {distance}')

        # Check if within 10 pixels of the target
        if distance < 10:
            self.get_logger().info('Target is within 10 pixels. Stopping tracking.')

            # Publish 'done' to 'target_box' topic
            done_msg = String()
            done_msg.data = 'done'
            self.target_publisher.publish(done_msg)
            self.get_logger().info("Published 'done' to 'target_box' topic.")

            # Publish to 'catch' topic
            self.publish_catch()

            # Stop tracking
            self.tracking_active = False

            # Start timer to restart tracking after 5 seconds
            self.get_logger().info('Starting timer to restart tracking in 5 seconds.')
            Timer(2.0, self.restart_tracking).start()

            return  # Exit the callback

        # Continue tracking and moving towards the target
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
        self.req.path_time = 0.5  # Path movement time (adjust as needed)

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

    def publish_catch(self):
        """Publish to the 'catch' topic."""
        msg = Empty()
        self.catch_publisher.publish(msg)
        self.get_logger().info('Published to catch topic.')

    def restart_tracking(self):
        """Restart tracking after the delay."""
        self.get_logger().info('Restarting tracking.')
        # Publish the target color to 'target_box' topic to resume tracking
        if self.target_color is not None:
            self.tracking_active = True
            resume_msg = String()
            resume_msg.data = self.target_color
            self.target_publisher.publish(resume_msg)
            self.get_logger().info(f"Published '{self.target_color}' to 'target_box' topic to resume tracking.")

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
