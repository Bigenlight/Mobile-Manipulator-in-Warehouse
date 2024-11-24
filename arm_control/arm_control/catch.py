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

        # 'goal_tool_control' 서비스 클라이언트 생성
        self.tool_control_cli = self.create_client(SetJointPosition, 'goal_tool_control')
        while not self.tool_control_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service goal_tool_control not available, waiting...')

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

        # 단계 관리 변수
        self.pickup_step = 0
        self.place_step = 0

        # 타이머 변수 초기화
        self.timer = None

        # 내려놓기 위한 목표 위치 설정 (절대 좌표값)
        self.place_pose1 = Pose()
        self.place_pose1.position.x = 0.013114313918879605
        self.place_pose1.position.y = 0.36321287339041936
        self.place_pose1.position.z = 0.025007863577571482
        self.place_pose1.orientation.x = -0.15310465745342147
        self.place_pose1.orientation.y = 0.15357509329946967
        self.place_pose1.orientation.z = 0.6892205764413497
        self.place_pose1.orientation.w = 0.6913383047350865

        self.place_pose2 = Pose()
        self.place_pose2.position.x = 0.16120536006233618
        self.place_pose2.position.y = -0.00022887834167865378
        self.place_pose2.position.z = 0.07502465855935364
        self.place_pose2.orientation.x = 0.0005526430781677229
        self.place_pose2.orientation.y = 0.7205343884158922
        self.place_pose2.orientation.z = -0.0005318456636457356
        self.place_pose2.orientation.w = 0.6934187817156053

        # 새로 추가된 pose3 정의
        self.place_pose3 = Pose()
        self.place_pose3.position.x = 0.24365265636623842
        self.place_pose3.position.y = 0.007464948005071252
        self.place_pose3.position.z = 0.1121231145123773
        self.place_pose3.orientation.x = -0.003105788896840778
        self.place_pose3.orientation.y = 0.19280803515108144
        self.place_pose3.orientation.z = 0.01580381674006639
        self.place_pose3.orientation.w = 0.98110430384998

    def kinematics_pose_callback(self, msg):
        self.current_pose = msg.pose

    def joint_states_callback(self, msg):
        self.current_joint_states = msg

    def catch_callback(self, msg):
        if self.pickup_step == 0 and self.place_step == 0:
            self.get_logger().info('Received catch signal, starting pickup sequence.')
            # 픽업 시퀀스 시작
            self.pickup_step = 1
            self.move_z_down()

    # 픽업 시퀀스 시작: z를 -0.015로 이동
    def move_z_down(self):
        self.get_logger().info('Pickup: Moving z to -0.015')
        task_space_req = SetKinematicsPose.Request()
        task_space_req.end_effector_name = 'gripper'
        task_space_req.kinematics_pose.pose.position.x = self.current_pose.position.x
        task_space_req.kinematics_pose.pose.position.y = self.current_pose.position.y
        task_space_req.kinematics_pose.pose.position.z = -0.015
        task_space_req.kinematics_pose.pose.orientation = self.current_pose.orientation
        task_space_req.path_time = 1.0  # 이동 시간 설정 (초 단위)

        self.get_logger().debug(f'Pickup: Sending move_z_down request: {task_space_req}')
        future = self.task_space_cli.call_async(task_space_req)
        future.add_done_callback(self.move_z_down_callback)

    def move_z_down_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Pickup: Command to move z=-0.015 sent successfully.')
            # 목표 위치에 도달했는지 확인하기 위해 타이머 시작
            self.target_z_pickup = -0.015
            self.timer = self.create_timer(0.1, self.check_position_pickup)
        except Exception as e:
            self.get_logger().error(f'Pickup: Failed to send command to move z=-0.015: {e}')
            self.reset_steps()

    def check_position_pickup(self):
        # 현재 z 위치와 목표 z 위치 비교
        current_z = self.current_pose.position.z
        self.get_logger().debug(f'Pickup: Checking position_pickup: current_z={current_z}, target_z={self.target_z_pickup}')
        if abs(current_z - self.target_z_pickup) < 0.01:  # 오차 허용 범위 1cm
            self.get_logger().info(f'Pickup: Target z={self.target_z_pickup} reached.')
            self.timer.cancel()
            self.timer = None
            if self.pickup_step == 1:
                self.pickup_step = 2
                self.close_gripper()

    # 그리퍼 닫기
    def close_gripper(self):
        self.get_logger().info('Pickup: Closing the gripper.')
        tool_control_req = SetJointPosition.Request()
        tool_control_req.joint_position.joint_name = ['gripper']
        tool_control_req.joint_position.position = [-0.01]  # 그리퍼 완전 닫기
        tool_control_req.path_time = 1.0

        self.get_logger().debug(f'Pickup: Sending close_gripper request: {tool_control_req}')
        future = self.tool_control_cli.call_async(tool_control_req)
        future.add_done_callback(self.close_gripper_callback)

    def close_gripper_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Pickup: Command to close gripper sent successfully.')
            # 그리퍼를 닫은 후 2초 대기 (타이머 사용)
            self.timer = self.create_timer(2.0, self.move_z_up_pickup)
        except Exception as e:
            self.get_logger().error(f'Pickup: Failed to send command to close gripper: {e}')
            self.reset_steps()

    # z를 0.05로 이동 (픽업 완료 후)
    def move_z_up_pickup(self):
        self.get_logger().info('Pickup: Moving z to 0.05')
        self.timer.cancel()
        self.timer = None

        task_space_req = SetKinematicsPose.Request()
        task_space_req.end_effector_name = 'gripper'
        task_space_req.kinematics_pose.pose.position.x = self.current_pose.position.x
        task_space_req.kinematics_pose.pose.position.y = self.current_pose.position.y
        task_space_req.kinematics_pose.pose.position.z = 0.05
        task_space_req.kinematics_pose.pose.orientation = self.current_pose.orientation
        task_space_req.path_time = 1.0

        self.get_logger().debug(f'Pickup: Sending move_z_up_pickup request: {task_space_req}')
        future = self.task_space_cli.call_async(task_space_req)
        future.add_done_callback(self.move_z_up_pickup_callback)

    def move_z_up_pickup_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Pickup: Command to move z=0.05 sent successfully.')
            self.target_z_pickup_up = 0.05
            self.timer = self.create_timer(0.1, self.check_position_pickup_up)
        except Exception as e:
            self.get_logger().error(f'Pickup: Failed to send command to move z=0.05: {e}')
            self.reset_steps()

    def check_position_pickup_up(self):
        current_z = self.current_pose.position.z
        self.get_logger().info(f'Pickup: Checking position_pickup_up: current_z={current_z}, target_z={self.target_z_pickup_up}')
        if abs(current_z - self.target_z_pickup_up) < 0.1:
            self.get_logger().info(f'Pickup: Target z={self.target_z_pickup_up} reached.')
            self.timer.cancel()
            self.timer = None
            if self.pickup_step == 2:
                self.pickup_step = 3
                self.move_to_place_pose1()

    # 내려놓기 시퀀스 시작: Pose 1으로 이동
    def move_to_place_pose1(self):
        self.get_logger().info('Place: Moving to target place pose 1.')
        task_space_req = SetKinematicsPose.Request()
        task_space_req.end_effector_name = 'gripper'
        task_space_req.kinematics_pose.pose = self.place_pose1
        task_space_req.path_time = 2.0  # 이동 시간 설정 (초 단위)

        self.get_logger().debug(f'Place: Sending move_to_place_pose1 request: {task_space_req}')
        future = self.task_space_cli.call_async(task_space_req)
        future.add_done_callback(self.move_to_place_pose1_callback)

    def move_to_place_pose1_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Place: Command to move to target place pose 1 sent successfully.')
            # 목표 위치에 도달했는지 확인하기 위해 타이머 시작
            self.target_z_place1 = self.place_pose1.position.z
            self.timer = self.create_timer(0.1, self.check_position_place1)
        except Exception as e:
            self.get_logger().error(f'Place: Failed to send command to move to target place pose 1: {e}')
            self.reset_steps()

    def check_position_place1(self):
        # 현재 z 위치와 목표 z 위치 비교
        current_z = self.current_pose.position.z
        self.get_logger().debug(f'Place: Checking position_place1: current_z={current_z}, target_z={self.target_z_place1}')
        if abs(current_z - self.target_z_place1) < 0.1:
            self.get_logger().info(f'Place: Target place pose 1 reached at z={self.target_z_place1}.')
            self.timer.cancel()
            self.timer = None
            if self.place_step == 0:
                self.place_step = 1
                # 그리퍼 열기 호출
                self.open_gripper()
                # 그리퍼 열기 후 0.5초 대기 (타이머 사용)
                self.timer = self.create_timer(0.5, self.move_to_place_pose3)
    
    # 그리퍼 열기
    def open_gripper(self):
        self.get_logger().info('Place: Opening the gripper.')
        tool_control_req = SetJointPosition.Request()
        tool_control_req.joint_position.joint_name = ['gripper']
        tool_control_req.joint_position.position = [0.01]  # 그리퍼 완전 열기
        tool_control_req.path_time = 1.0

        self.get_logger().debug(f'Place: Sending open_gripper request: {tool_control_req}')
        future = self.tool_control_cli.call_async(tool_control_req)
        future.add_done_callback(self.open_gripper_callback)

    def open_gripper_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Place: Command to open gripper sent successfully.')
            # 그리퍼를 연 후 추가 작업 없음
        except Exception as e:
            self.get_logger().error(f'Place: Failed to send command to open gripper: {e}')
            self.reset_steps()

    # 새로 추가된 함수: pose3으로 이동
    def move_to_place_pose3(self):
        self.get_logger().info('Place: Moving to target place pose 3.')
        self.timer.cancel()
        self.timer = None

        task_space_req = SetKinematicsPose.Request()
        task_space_req.end_effector_name = 'gripper'
        task_space_req.kinematics_pose.pose = self.place_pose3
        task_space_req.path_time = 2.0  # 이동 시간 설정 (초 단위)

        self.get_logger().debug(f'Place: Sending move_to_place_pose3 request: {task_space_req}')
        future = self.task_space_cli.call_async(task_space_req)
        future.add_done_callback(self.move_to_place_pose3_callback)

    def move_to_place_pose3_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Place: Command to move to target place pose 3 sent successfully.')
            # 목표 위치에 도달했는지 확인하기 위해 타이머 시작
            self.target_z_place3 = self.place_pose3.position.z
            self.timer = self.create_timer(0.1, self.check_position_place3)
        except Exception as e:
            self.get_logger().error(f'Place: Failed to send command to move to target place pose 3: {e}')
            self.reset_steps()

    def check_position_place3(self):
        # 현재 z 위치와 목표 z 위치 비교
        current_z = self.current_pose.position.z
        self.get_logger().debug(f'Place: Checking position_place3: current_z={current_z}, target_z={self.target_z_place3}')
        if abs(current_z - self.target_z_place3) < 0.01:  # 오차 허용 범위 1cm
            self.get_logger().info(f'Place: Target place pose 3 reached at z={self.target_z_place3}.')
            self.timer.cancel()
            self.timer = None
            if self.place_step == 1:
                self.place_step = 2
                self.move_to_place_pose2()

    # 내려놓기 시퀀스 시작: Pose 2으로 이동
    def move_to_place_pose2(self):
        self.get_logger().info('Place: Moving to target place pose 2.')
        task_space_req = SetKinematicsPose.Request()
        task_space_req.end_effector_name = 'gripper'
        task_space_req.kinematics_pose.pose = self.place_pose2
        task_space_req.path_time = 2.0  # 이동 시간 설정 (초 단위)

        self.get_logger().debug(f'Place: Sending move_to_place_pose2 request: {task_space_req}')
        future = self.task_space_cli.call_async(task_space_req)
        future.add_done_callback(self.move_to_place_pose2_callback)

    def move_to_place_pose2_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Place: Command to move to target place pose 2 sent successfully.')
            # 목표 위치에 도달했는지 확인하기 위해 타이머 시작
            self.target_z_place2 = self.place_pose2.position.z
            self.timer = self.create_timer(0.1, self.check_position_place2)
        except Exception as e:
            self.get_logger().error(f'Place: Failed to send command to move to target place pose 2: {e}')
            self.reset_steps()

    def check_position_place2(self):
        # 현재 z 위치와 목표 z 위치 비교
        current_z = self.current_pose.position.z
        self.get_logger().debug(f'Place: Checking position_place2: current_z={current_z}, target_z={self.target_z_place2}')
        if abs(current_z - self.target_z_place2) < 0.01:  # 오차 허용 범위 1cm
            self.get_logger().info(f'Place: Target place pose 2 reached at z={self.target_z_place2}.')
            self.timer.cancel()
            self.timer = None
            if self.place_step == 2:
                self.place_step = 3
                # 그리퍼 열기 호출
                self.open_gripper()
                # 그리퍼 열기 후 0.5초 대기 (타이머 사용)
                self.timer = self.create_timer(0.5, self.sequence_completed)

    def sequence_completed(self):
        self.get_logger().info('Box placed successfully. Release sequence completed.')
        self.timer.cancel()
        self.timer = None
        self.reset_steps()

    def reset_steps(self):
        self.pickup_step = 0
        self.place_step = 0
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        self.get_logger().info('Sequence reset.')

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
