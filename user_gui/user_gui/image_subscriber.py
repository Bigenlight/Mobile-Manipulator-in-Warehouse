#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import os
import sys
from ultralytics import YOLO

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # 이미지 토픽 구독
        self.image_subscription = self.create_subscription(
            Image,
            '/image_raw/compressed',  # 퍼블리셔의 새로운 토픽 이름과 일치
            self.listener_callback_image,
            10)
        self.image_subscription  # 사용되지 않는 변수 경고 방지

        self.bridge = CvBridge()
        self.latest_ros_frame = None
        self.lock_image = threading.Lock()
        self.frame_available = threading.Event()

        self.get_logger().info('ImageSubscriber 노드가 시작되었습니다.')

        # YOLOv8 모델 파일 경로 설정
        model_path = '/home/rokey/3_ws/src/best.pt'  # 사용자 지정 모델 파일 경로

        # YOLOv8 모델 파일 존재 여부 확인
        if not os.path.isfile(model_path):
            self.get_logger().fatal(f'YOLOv8 모델 파일을 찾을 수 없습니다: {model_path}')
            rclpy.shutdown()
            sys.exit(1)

        # YOLOv8 모델 로드
        try:
            self.model = YOLO(model_path)  # YOLOv8 모델 로드
            self.get_logger().info('YOLOv8 모델이 성공적으로 로드되었습니다.')
        except Exception as e:
            self.get_logger().fatal(f'YOLOv8 모델 로드 오류: {e}')
            rclpy.shutdown()
            sys.exit(1)

        # 별도의 스레드에서 이미지 표시
        self.display_thread = threading.Thread(target=self.display_images)
        self.display_thread.daemon = True
        self.display_thread.start()

    def listener_callback_image(self, msg):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock_image:
                self.latest_ros_frame = cv_image.copy()
            self.frame_available.set()
            self.get_logger().debug('image_raw/compressed 토픽에서 새로운 프레임을 수신했습니다.')
        except Exception as e:
            self.get_logger().error(f'image_raw/compressed 이미지 변환 오류: {e}')

    def display_images(self):
        while rclpy.ok():
            # 새로운 프레임이 도착할 때까지 대기
            if self.frame_available.wait(timeout=1.0):
                with self.lock_image:
                    frame = self.latest_ros_frame.copy() if self.latest_ros_frame is not None else None
                self.frame_available.clear()

                if frame is not None:
                    try:
                        # YOLOv8을 사용한 객체 검출
                        results = self.model(frame)

                        # 결과 파싱 및 바운딩 박스 그리기
                        for result in results:
                            boxes = result.boxes  # Boxes 객체
                            for box in boxes:
                                # 바운딩 박스 좌표 및 신뢰도
                                x1, y1, x2, y2 = box.xyxy[0]
                                confidence = box.conf[0]
                                class_id = int(box.cls[0])
                                class_name = self.model.names[class_id]

                                # 좌표를 정수로 변환
                                x1 = int(x1)
                                y1 = int(y1)
                                x2 = int(x2)
                                y2 = int(y2)
                                confidence = float(confidence)

                                # 바운딩 박스 그리기
                                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                
                                # 클래스 이름 및 신뢰도 표시
                                label = f"{class_name} {confidence:.2f}"
                                cv2.putText(frame, label, (x1, y1 - 25), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                
                                # 바운딩 박스의 중심 좌표 계산
                                center_x = int((x1 + x2) / 2)
                                center_y = int((y1 + y2) / 2)
                                
                                # 중심 좌표에 작은 원 그리기
                                cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)
                                
                                # 중심 좌표 표시 (이미지에 텍스트 추가)
                                coord_label = f"({center_x}, {center_y})"
                                cv2.putText(frame, coord_label, (center_x + 10, center_y), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                                # ROS2 로그에 중심 좌표 출력
                                self.get_logger().info(
                                    f"Detected {class_name} with confidence {confidence:.2f} at "
                                    f"({center_x}, {center_y})"
                                )

                        # 이미지 창에 표시
                        cv2.imshow('YOLOv8 Detection', frame)
                        # 키 입력 대기 (1ms)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            self.get_logger().info('이미지 표시를 종료합니다.')
                            rclpy.shutdown()
                            break
                    except Exception as e:
                        self.get_logger().error(f'YOLOv8 처리 중 오류: {e}')
            else:
                self.get_logger().warning('프레임 수신 대기 중 타임아웃 발생.')

    def destroy_node(self):
        self.get_logger().info('ImageSubscriber 노드를 종료합니다.')
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('키보드 인터럽트(SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
