#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetKinematicsPose
from geometry_msgs.msg import Pose
import time

class MoveToPose2Node(Node):
    def __init__(self):
        super().__init__('move_to_pose2_node')

        # Create a client for the 'goal_task_space_path' service
        self.cli = self.create_client(SetKinematicsPose, 'goal_task_space_path')

        self.get_logger().info('Waiting for service "goal_task_space_path"...')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service "goal_task_space_path" not available, waiting...')

        # Once the service is available, send the pose request
        self.send_goal()

    def send_goal(self):
        # Create a request for SetKinematicsPose
        request = SetKinematicsPose.Request()

        # Define the end effector name (ensure this matches your manipulator's configuration)
        # Common names are 'gripper', 'end_effector', or 'tool'—adjust as necessary
        request.end_effector_name = 'gripper'  # Replace with the correct name if different

        # Define the target pose (place_pose2)
        request.kinematics_pose.pose = self.get_place_pose2()

        # Set the path time (duration to reach the target pose in seconds)
        request.path_time = 5.0  # Increased time for clarity

        self.get_logger().info('Sending request to move to place_pose2...')
        self.get_logger().debug(f'Request Pose: {request.kinematics_pose.pose}')

        # Call the service synchronously
        self.call_service(request)

    def get_place_pose2(self):
        # Define place_pose2 as per your specification
        pose2 = Pose()
        pose2.position.x = 0.16120536006233618
        pose2.position.y = -0.00022887834167865378
        pose2.position.z = 0.07502465855935364
        pose2.orientation.x = 0.0005526430781677229
        pose2.orientation.y = 0.7205343884158922
        pose2.orientation.z = -0.0005318456636457356
        pose2.orientation.w = 0.6934187817156053
        return pose2

    def call_service(self, request):
        try:
            # Make a synchronous service call
            future = self.cli.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.done():
                response = future.result()
                if response.is_planned:
                    self.get_logger().info('Successfully planned movement to place_pose2.')
                    # Wait for the manipulator to execute the movement
                    self.get_logger().info('Waiting for manipulator to move...')
                    time.sleep(request.path_time + 2)  # Wait for movement to complete
                    self.get_logger().info('Movement should be completed.')
                else:
                    self.get_logger().error('Failed to plan movement to place_pose2.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        finally:
            # Shutdown the node after handling the response
            self.get_logger().info('Shutting down node.')
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MoveToPose2Node()
    # No need to spin since we're handling everything synchronously
    # rclpy.spin(node)  # Commented out to prevent premature shutdown

if __name__ == '__main__':
    main()
