#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetKinematicsPose
from open_manipulator_msgs.msg import KinematicsPose
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from threading import Event
import re
import json
from datetime import datetime

class MoveUpwardClient(Node):
    def __init__(self):
        super().__init__('move_upward_client')
        
        # 파라미터 선언 (타겟 좌표 및 상수)
        self.declare_parameter('target_x', 320)  # 예시 값, 필요에 따라 수정 가능
        self.declare_parameter('target_y', 240)  # 예시 값, 필요에 따라 수정 가능
        self.declare_parameter('kx', -0.0003328895)
        self.declare_parameter('ky', -0.0003328895)
        self.declare_parameter('fixed_z', 0.11)  # 고정할 Z축 값
        
        # 파라미터 값 읽기
        self.target_x = self.get_parameter('target_x').get_parameter_value().integer_value
        self.target_y = self.get_parameter('target_y').get_parameter_value().integer_value
        self.kx = self.get_parameter('kx').get_parameter_value().double_value
        self.ky = self.get_parameter('ky').get_parameter_value().double_value
        self.fixed_z = self.get_parameter('fixed_z').get_parameter_value().double_value

        # 서비스 클라이언트 생성
        self.cli = self.create_client(SetKinematicsPose, 'goal_task_space_path')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스가 사용 가능하지 않습니다. 대기 중...')
        self.req = SetKinematicsPose.Request()

        # 현재 포즈 저장 변수
        self.current_pose = Pose()
        self.pose_received = Event()

        # 현재 포즈를 받기 위한 구독자 생성
        self.subscription_pose = self.create_subscription(
            KinematicsPose,
            'kinematics_pose',
            self.kinematics_pose_callback,
            10)
        self.subscription_pose  # prevent unused variable warning

        # /pixel_coord 토픽 구독자 생성
        self.subscription_pixel = self.create_subscription(
            String,
            '/pixel_coord',
            self.pixel_coord_callback,
            10)
        self.subscription_pixel  # prevent unused variable warning

        self.get_logger().info('MoveUpwardClient 노드가 시작되었습니다.')

    def kinematics_pose_callback(self, msg):
        """현재 매니퓰레이터의 포즈를 업데이트."""
        self.current_pose = msg.pose
        self.pose_received.set()
        self.get_logger().debug(f'현재 포즈 수신: x={self.current_pose.position.x}, y={self.current_pose.position.y}, z={self.current_pose.position.z}')

    def pixel_coord_callback(self, msg):
        """/pixel_coord 토픽으로부터 픽셀 좌표를 수신하고, 매니퓰레이터 위치를 업데이트."""
        # 메시지에서 p_x와 p_y 추출
        p_x, p_y = self.parse_pixel_coord(msg.data)
        if p_x is None or p_y is None:
            self.get_logger().error(f'픽셀 좌표 파싱 실패: {msg.data}')
            return

        self.get_logger().info(f'픽셀 좌표 수신: p_x={p_x}, p_y={p_y}')

        # p_dx와 p_dy 계산
        p_dx = p_x - self.target_x
        p_dy = p_y - self.target_y
        self.get_logger().debug(f'p_dx={p_dx}, p_dy={p_dy}')

        # dx와 dy 계산
        dx = self.kx * p_dx
        dy = self.ky * p_dy
        self.get_logger().debug(f'dx={dx}, dy={dy}')

        # 현재 포즈가 수신되었는지 확인
        if not self.pose_received.is_set():
            self.get_logger().info('현재 포즈를 기다리는 중...')
            self.pose_received.wait(timeout=5.0)
            if not self.pose_received.is_set():
                self.get_logger().error('타임아웃: 현재 포즈를 수신하지 못했습니다.')
                return

        # 새로운 목표 포즈 설정
        self.req.end_effector_name = 'gripper'  # 매니퓰레이터의 엔드 이펙터 이름으로 변경 필요
        self.req.kinematics_pose.pose.position.x = self.current_pose.position.x + dx
        self.req.kinematics_pose.pose.position.y = self.current_pose.position.y + dy
        self.req.kinematics_pose.pose.position.z = self.fixed_z  # Z축을 0.11로 고정
        self.req.kinematics_pose.pose.orientation = self.current_pose.orientation  # 방향 동일하게 유지
        self.req.path_time = 0.5  # 경로 이동 시간 (필요에 따라 조정)

        self.get_logger().info(f'목표 포즈 설정: x={self.req.kinematics_pose.pose.position.x}, y={self.req.kinematics_pose.pose.position.y}, z={self.req.kinematics_pose.pose.position.z}')

        # 비동기로 서비스 요청 보내기
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.service_response_callback)

    def parse_pixel_coord(self, data):
        """
        문자열 형식 '(300, 420)'에서 정수 p_x와 p_y를 추출.
        실패 시 (None, None) 반환.
        """
        pattern = r'\(\s*(\d+)\s*,\s*(\d+)\s*\)'
        match = re.match(pattern, data)
        if match:
            p_x = int(match.group(1))
            p_y = int(match.group(2))
            return p_x, p_y
        else:
            return None, None

    def service_response_callback(self, future):
        """서비스 응답 처리."""
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'서비스 호출 실패: {e}')
        else:
            if response.is_planned:
                self.get_logger().info('매니퓰레이터가 성공적으로 이동했습니다.')
            else:
                self.get_logger().error('매니퓰레이터 이동 실패.')

    def destroy_node(self):
        """노드 종료 시 자원 정리."""
        self.get_logger().info('MoveUpwardClient 노드가 종료됩니다.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    client = MoveUpwardClient()

    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
