#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetKinematicsPose, SetJointPosition
from open_manipulator_msgs.msg import KinematicsPose
from geometry_msgs.msg import Pose
from threading import Event
from std_msgs.msg import Empty
import time

class MoveUpwardClient(Node):
    def __init__(self):
        super().__init__('move_upward_client')

        # Create client to 'goal_task_space_path' service
        self.cli = self.create_client(SetKinematicsPose, 'goal_task_space_path')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service goal_task_space_path not available, waiting...')
        self.req = SetKinematicsPose.Request()

        # Create client to 'goal_tool_control' service
        self.tool_cli = self.create_client(SetJointPosition, 'goal_tool_control')
        while not self.tool_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service goal_tool_control not available, waiting...')

        # Create publisher to 'convayor' topic
        self.convayor_publisher = self.create_publisher(Empty, 'convayor', 10)

        # Subscribe to 'kinematics_pose' to get current end-effector pose
        self.current_pose = Pose()
        self.pose_received = Event()
        self.subscription = self.create_subscription(
            KinematicsPose,
            'kinematics_pose',
            self.kinematics_pose_callback,
            10)

        # Subscribe to 'catch' topic
        self.catch_subscription = self.create_subscription(
            Empty,
            'catch',
            self.catch_callback,
            10)

    def kinematics_pose_callback(self, msg):
        self.current_pose = msg.pose
        self.pose_received.set()

    def catch_callback(self, msg):
        self.get_logger().info('Received catch signal, starting sequence.')

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

        # 1. Move z to 0.03
        self.get_logger().info('Moving z to 0.03')
        self.req.end_effector_name = 'gripper'
        self.req.kinematics_pose.pose.position.x = self.current_pose.position.x
        self.req.kinematics_pose.pose.position.y = self.current_pose.position.y
        self.req.kinematics_pose.pose.position.z = 0.03
        self.req.kinematics_pose.pose.orientation = self.current_pose.orientation
        self.req.path_time = 0.5

        move_future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, move_future)
        if move_future.done():
            try:
                response = move_future.result()
            except Exception as e:
                self.get_logger().error(f'Failed to move to z=0.03: {e}')
                return
            else:
                self.get_logger().info('Moved to z=0.03 successfully.')

        # 2. Close the gripper completely
        self.get_logger().info('Closing the gripper.')
        tool_req = SetJointPosition.Request()
        tool_req.joint_position.joint_name = ['gripper']
        tool_req.joint_position.position = [-0.01]  # Fully close the gripper
        tool_req.path_time = 1.0

        tool_future = self.tool_cli.call_async(tool_req)
        rclpy.spin_until_future_complete(self, tool_future)
        if tool_future.done():
            try:
                response = tool_future.result()
            except Exception as e:
                self.get_logger().error(f'Failed to close gripper: {e}')
                return
            else:
                self.get_logger().info('Gripper closed successfully.')

        # Wait for 1 second
        self.get_logger().info('Waiting for 1 second.')
        time.sleep(1)

        # 3. Move z to 0.15
        self.get_logger().info('Moving z to 0.15')
        self.req.kinematics_pose.pose.position.z = 0.15
        move_future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, move_future)
        if move_future.done():
            try:
                response = move_future.result()
            except Exception as e:
                self.get_logger().error(f'Failed to move to z=0.15: {e}')
                return
            else:
                self.get_logger().info('Moved to z=0.15 successfully.')

        # 4. Publish to 'convayor' topic
        self.get_logger().info('Publishing to convayor topic.')
        convayor_msg = Empty()
        self.convayor_publisher.publish(convayor_msg)

        self.get_logger().info('Sequence completed.')

def main(args=None):
    rclpy.init(args=args)
    client = MoveUpwardClient()
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
