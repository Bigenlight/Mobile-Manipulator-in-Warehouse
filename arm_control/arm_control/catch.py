#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetKinematicsPose, SetJointPosition
from open_manipulator_msgs.msg import KinematicsPose
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from threading import Event
from std_msgs.msg import Empty

import time
import math

class MoveUpwardClient(Node):
    def __init__(self):
        super().__init__('move_upward_client')

        # 'goal_task_space_path' 서비스 클라이언트 생성
        self.task_space_cli = self.create_client(SetKinematicsPose, 'goal_task_space_path')
        while not self.task_space_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service goal_task_space_path not available, waiting...')
        self.task_space_req = SetKinematicsPose.Request()

        # 'goal_tool_control' 서비스 클라이언트 생성
        self.tool_control_cli = self.create_client(SetJointPosition, 'goal_tool_control')
        while not self.tool_control_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service goal_tool_control not available, waiting...')
        self.tool_control_req = SetJointPosition.Request()

        # 'convayor' 토픽 퍼블리셔 생성
        self.convayor_publisher = self.create_publisher(Empty, 'convayor', 10)

        # 현재 엔드 이펙터 포즈 및 그리퍼 상태 구독
        self.current_pose = Pose()
        self.current_joint_states = JointState()
        self.pose_subscription = self.create_subscription(
            KinematicsPose,
            'kinematics_pose',
            self.kinematics_pose_callback,
            10)
        self.joint_states_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10)

        # 'catch' 토픽 구독
        self.catch_subscription = self.create_subscription(
            Empty,
            'catch',
            self.catch_callback,
            10)

        # 그리퍼 제어를 위한 변수 초기화
        self.goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]

        # 단계 관리 변수
        self.step = 0
        self.target_reached = False
        self.gripper_closed = False

    def kinematics_pose_callback(self, msg):
        self.current_pose = msg.pose

    def joint_states_callback(self, msg):
        self.current_joint_states = msg

    def catch_callback(self, msg):
        self.get_logger().info('Received catch signal, starting sequence.')
        # 시퀀스 시작
        self.step = 1
        self.move_z_down()

    def move_z_down(self):
        # 1. z를 -0.015로 이동
        self.get_logger().info('Moving z to -0.015')
        self.task_space_req.end_effector_name = 'gripper'
        self.task_space_req.kinematics_pose.pose.position.x = self.current_pose.position.x
        self.task_space_req.kinematics_pose.pose.position.y = self.current_pose.position.y
        self.task_space_req.kinematics_pose.pose.position.z = -0.015
        self.task_space_req.kinematics_pose.pose.orientation = self.current_pose.orientation
        self.task_space_req.path_time = 1.0

        future = self.task_space_cli.call_async(self.task_space_req)
        future.add_done_callback(self.move_z_down_callback)

    def move_z_down_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Command to move z=-0.015 sent successfully.')
            # 목표 위치에 도달했는지 확인하기 위해 타이머 시작
            self.target_z = -0.015
            self.target_reached = False
            self.check_position_timer = self.create_timer(0.1, self.check_position)
        except Exception as e:
            self.get_logger().error(f'Failed to send command to move z=-0.015: {e}')

    def check_position(self):
        # 현재 z 위치와 목표 z 위치 비교
        current_z = self.current_pose.position.z
        if abs(current_z - self.target_z) < 0.005:  # 오차 허용 범위 5mm
            self.get_logger().info(f'Target z={self.target_z} reached.')
            self.target_reached = True
            self.check_position_timer.cancel()
            if self.step == 1:
                self.step = 2
                self.close_gripper()
            elif self.step == 3:
                self.step = 4
                self.publish_convayor()

    def close_gripper(self):
        self.get_logger().info('Closing the gripper.')
        self.tool_control_req.joint_position.joint_name = ['gripper']
        self.tool_control_req.joint_position.position = [-0.01]  # 그리퍼 완전 닫기
        self.tool_control_req.path_time = 1.0

        future = self.tool_control_cli.call_async(self.tool_control_req)
        future.add_done_callback(self.close_gripper_callback)

    def close_gripper_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Command to close gripper sent successfully.')
            # 그리퍼가 닫혔는지 확인하기 위해 타이머 시작
            self.target_gripper_position = -0.01
            self.gripper_closed = False
            self.check_gripper_timer = self.create_timer(0.1, self.check_gripper_position)
        except Exception as e:
            self.get_logger().error(f'Failed to send command to close gripper: {e}')

    def check_gripper_position(self):
        # 현재 그리퍼 위치와 목표 그리퍼 위치 비교
        gripper_index = self.current_joint_states.name.index('gripper')
        current_gripper_position = self.current_joint_states.position[gripper_index]
        if abs(current_gripper_position - self.target_gripper_position) < 1:  # 오차 허용 범위
            self.get_logger().info('Gripper closed successfully.')
            self.gripper_closed = True
            self.check_gripper_timer.cancel()
            if self.step == 2:
                self.step = 3
                self.move_z_up()

    def move_z_up(self):
        self.get_logger().info('Moving z to 0.15')
        self.task_space_req.kinematics_pose.pose.position.z = 0.15
        self.task_space_req.path_time = 1.0

        future = self.task_space_cli.call_async(self.task_space_req)
        future.add_done_callback(self.move_z_up_callback)

    def move_z_up_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Command to move z=0.15 sent successfully.')
            # 목표 위치에 도달했는지 확인하기 위해 타이머 시작
            self.target_z = 0.15
            self.target_reached = False
            self.check_position_timer = self.create_timer(0.1, self.check_position)
        except Exception as e:
            self.get_logger().error(f'Failed to send command to move z=0.15: {e}')

    def publish_convayor(self):
        self.get_logger().info('Publishing to convayor topic.')
        convayor_msg = Empty()
        self.convayor_publisher.publish(convayor_msg)
        self.get_logger().info('Sequence completed.')
        self.step = 0  # 시퀀스 완료

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
