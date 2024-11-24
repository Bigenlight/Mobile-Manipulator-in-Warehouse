#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetKinematicsPose, SetJointPosition
from open_manipulator_msgs.msg import KinematicsPose
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty

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

        # 'catch' 토픽 구독 (픽업 트리거)
        self.catch_subscription = self.create_subscription(
            Empty,
            'catch',
            self.catch_callback,
            10)

        # 그리퍼 제어를 위한 변수 초기화
        self.goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]

        # 단계 관리 변수
        self.step = 0

        # 대기 타이머 변수 초기화
        self.wait_timer = None
        self.check_position_timer = None

        # 내려놓기 위한 목표 위치 설정 (절대 좌표값)
        self.place_pose1 = Pose()
        self.place_pose1.position.x = 0.015939466018714724
        self.place_pose1.position.y = 0.28533002786194017
        self.place_pose1.position.z = 0.07140051770579621
        self.place_pose1.orientation.x = -0.2855484931845922
        self.place_pose1.orientation.y = 0.289518190526194
        self.place_pose1.orientation.z = 0.6415289284640359
        self.place_pose1.orientation.w = 0.6504474685462563

        self.place_pose2 = Pose()
        self.place_pose2.position.x = 0.16120536006233618
        self.place_pose2.position.y = -0.00022887834167865378
        self.place_pose2.position.z = 0.07502465855935364
        self.place_pose2.orientation.x = 0.0005526430781677229
        self.place_pose2.orientation.y = 0.7205343884158922
        self.place_pose2.orientation.z = -0.0005318456636457356
        self.place_pose2.orientation.w = 0.6934187817156053

    def kinematics_pose_callback(self, msg):
        self.current_pose = msg.pose

    def joint_states_callback(self, msg):
        self.current_joint_states = msg

    def catch_callback(self, msg):
        if self.step == 0:
            self.get_logger().info('Received catch signal, starting pickup sequence.')
            # 시퀀스 시작
            self.step = 1
            self.move_z_down()

    # 픽업 시퀀스 시작: z를 -0.015로 이동
    def move_z_down(self):
        self.get_logger().info('Moving z to -0.015')
        self.task_space_req.end_effector_name = 'gripper'
        self.task_space_req.kinematics_pose.pose.position.x = self.current_pose.position.x
        self.task_space_req.kinematics_pose.pose.position.y = self.current_pose.position.y
        self.task_space_req.kinematics_pose.pose.position.z = -0.015
        self.task_space_req.kinematics_pose.pose.orientation = self.current_pose.orientation
        self.task_space_req.path_time = 1.0  # 이동 시간 설정 (초 단위)

        future = self.task_space_cli.call_async(self.task_space_req)
        future.add_done_callback(self.move_z_down_callback)

    def move_z_down_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Command to move z=-0.015 sent successfully.')
            # 목표 위치에 도달했는지 확인하기 위해 타이머 시작
            self.target_z_pickup = -0.015
            self.check_position_timer_pickup = self.create_timer(0.1, self.check_position_pickup)
        except Exception as e:
            self.get_logger().error(f'Failed to send command to move z=-0.015: {e}')
            self.step = 0  # 시퀀스 초기화

    def check_position_pickup(self):
        # 현재 z 위치와 목표 z 위치 비교
        current_z = self.current_pose.position.z
        if abs(current_z - self.target_z_pickup) < 0.005:  # 오차 허용 범위 5mm
            self.get_logger().info(f'Target z={self.target_z_pickup} reached.')
            self.check_position_timer_pickup.cancel()
            if self.step == 1:
                self.step = 2
                self.close_gripper()

    # 그리퍼 닫기
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
            # 그리퍼를 닫은 후 2초 대기
            self.wait_timer = self.create_timer(2.0, self.timer_callback_move_z_up)
        except Exception as e:
            self.get_logger().error(f'Failed to send command to close gripper: {e}')
            self.step = 0  # 시퀀스 초기화

    def timer_callback_move_z_up(self):
        self.move_z_up()
        if self.wait_timer is not None:
            self.wait_timer.cancel()
            self.wait_timer = None

    # z를 0.15로 이동 (픽업 완료 후)
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
            self.target_z_pickup_up = 0.15
            self.check_position_timer_pickup_up = self.create_timer(0.1, self.check_position_pickup_up)
        except Exception as e:
            self.get_logger().error(f'Failed to send command to move z=0.15: {e}')
            self.step = 0  # 시퀀스 초기화

    def check_position_pickup_up(self):
        # 현재 z 위치와 목표 z 위치 비교
        current_z = self.current_pose.position.z
        if abs(current_z - self.target_z_pickup_up) < 0.005:  # 오차 허용 범위 5mm
            self.get_logger().info(f'Target z={self.target_z_pickup_up} reached.')
            self.check_position_timer_pickup_up.cancel()
            if self.step == 2:
                self.step = 3
                self.move_to_place_pose1()

    # 내려놓기 시퀀스 시작: Pose 1으로 이동
    def move_to_place_pose1(self):
        self.get_logger().info('Moving to target place pose 1.')
        self.task_space_req.end_effector_name = 'gripper'
        self.task_space_req.kinematics_pose.pose = self.place_pose1
        self.task_space_req.path_time = 2.0  # 이동 시간 설정 (초 단위)

        future = self.task_space_cli.call_async(self.task_space_req)
        future.add_done_callback(self.move_to_place_pose1_callback)

    def move_to_place_pose1_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Command to move to target place pose 1 sent successfully.')
            # 목표 위치에 도달했는지 확인하기 위해 타이머 시작
            self.target_z_place1 = self.place_pose1.position.z
            self.check_position_timer_place1 = self.create_timer(0.1, self.check_position_place1)
        except Exception as e:
            self.get_logger().error(f'Failed to send command to move to target place pose 1: {e}')
            self.step = 0  # 시퀀스 초기화

    def check_position_place1(self):
        # 현재 z 위치와 목표 z 위치 비교
        current_z = self.current_pose.position.z
        if abs(current_z - self.target_z_place1) < 0.005:  # 오차 허용 범위 5mm
            self.get_logger().info(f'Target place pose 1 reached at z={self.target_z_place1}.')
            self.check_position_timer_place1.cancel()
            if self.step == 3:
                self.step = 4
                self.move_to_place_pose2()

    # 내려놓기 시퀀스 시작: Pose 2으로 이동
    def move_to_place_pose2(self):
        self.get_logger().info('Moving to target place pose 2.')
        self.task_space_req.end_effector_name = 'gripper'
        self.task_space_req.kinematics_pose.pose = self.place_pose2
        self.task_space_req.path_time = 2.0  # 이동 시간 설정 (초 단위)

        future = self.task_space_cli.call_async(self.task_space_req)
        future.add_done_callback(self.move_to_place_pose2_callback)

    def move_to_place_pose2_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Command to move to target place pose 2 sent successfully.')
            # 목표 위치에 도달했는지 확인하기 위해 타이머 시작
            self.target_z_place2 = self.place_pose2.position.z
            self.check_position_timer_place2 = self.create_timer(0.1, self.check_position_place2)
        except Exception as e:
            self.get_logger().error(f'Failed to send command to move to target place pose 2: {e}')
            self.step = 0  # 시퀀스 초기화

    def check_position_place2(self):
        # 현재 z 위치와 목표 z 위치 비교
        current_z = self.current_pose.position.z
        if abs(current_z - self.target_z_place2) < 0.005:  # 오차 허용 범위 5mm
            self.get_logger().info(f'Target place pose 2 reached at z={self.target_z_place2}.')
            self.check_position_timer_place2.cancel()
            if self.step == 4:
                self.step = 5
                self.open_gripper()

    # 그리퍼 열기
    def open_gripper(self):
        self.get_logger().info('Opening the gripper.')
        self.tool_control_req.joint_position.joint_name = ['gripper']
        self.tool_control_req.joint_position.position = [0.01]  # 그리퍼 완전 열기
        self.tool_control_req.path_time = 1.0

        future = self.tool_control_cli.call_async(self.tool_control_req)
        future.add_done_callback(self.open_gripper_callback)

    def open_gripper_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Command to open gripper sent successfully.')
            # 그리퍼를 연 후 1초 대기
            self.wait_timer = self.create_timer(1.0, self.timer_callback_sequence_completed)
        except Exception as e:
            self.get_logger().error(f'Failed to send command to open gripper: {e}')
            self.step = 0  # 시퀀스 초기화

    def timer_callback_sequence_completed(self):
        self.sequence_completed()
        if self.wait_timer is not None:
            self.wait_timer.cancel()
            self.wait_timer = None

    # 내려놓기 완료 확인
    def sequence_completed(self):
        self.get_logger().info('Box placed successfully. Release sequence completed.')
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
