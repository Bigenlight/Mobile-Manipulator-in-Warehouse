#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetKinematicsPose
from open_manipulator_msgs.msg import KinematicsPose
from geometry_msgs.msg import Pose
from threading import Event

class MoveUpwardClient(Node):
    def __init__(self):
        super().__init__('move_upward_client')
        self.cli = self.create_client(SetKinematicsPose, 'goal_task_space_path')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = SetKinematicsPose.Request()

        self.current_pose = Pose()
        self.pose_received = Event()

        # Subscribe to kinematics_pose to get the current end-effector pose
        self.subscription = self.create_subscription(
            KinematicsPose,
            'kinematics_pose',
            self.kinematics_pose_callback,
            10)
        self.subscription  # prevent unused variable warning

    def kinematics_pose_callback(self, msg):
        self.current_pose = msg.pose
        self.pose_received.set()

    def send_request(self):
        # Wait until the current pose is received
        self.get_logger().info('Waiting for current pose...')

        timeout_sec = 5.0
        start_time = self.get_clock().now()
        while not self.pose_received.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed_time > timeout_sec:
                self.get_logger().error('Failed to receive current pose.')
                return

        self.get_logger().info('Sending request to move upward by 0.01m')

        # Set the new goal pose by increasing z by 0.01
        self.req.end_effector_name = 'gripper'
        self.req.kinematics_pose.pose.position.x = self.current_pose.position.x
        self.req.kinematics_pose.pose.position.y = self.current_pose.position.y
        self.req.kinematics_pose.pose.position.z = self.current_pose.position.z - 0.05  # Move up by 0.01m
        self.req.kinematics_pose.pose.orientation = self.current_pose.orientation
        self.req.path_time = 0.5  # Adjust as needed

        self.future = self.cli.call_async(self.req)

    def wait_for_result(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result()
                except Exception as e:
                    self.get_logger().info(f'Service call failed: {e}')
                else:
                    self.get_logger().info('Moved upward successfully.')
                break

def main(args=None):
    rclpy.init(args=args)

    client = MoveUpwardClient()

    client.send_request()
    client.wait_for_result()

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
