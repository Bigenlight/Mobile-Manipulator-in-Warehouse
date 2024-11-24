import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # detection_info를 위한 String 메시지 타입 임포트
from cv_bridge import CvBridge
import cv2
import threading
import numpy as np
import time
import json  # detection_info 파싱을 위해 임포트

# ROS 이미지 구독자 노드
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # 이미지 토픽 구독
        self.image_subscription = self.create_subscription(
            Image,
            'image_raw',  # 퍼블리셔의 토픽 이름과 일치
            self.listener_callback_image,
            10)
        self.image_subscription  # 사용되지 않는 변수 경고 방지

        # detection_info 토픽 구독
        self.detection_subscription = self.create_subscription(
            String,
            'detection_info',
            self.listener_callback_detection,
            10)
        self.detection_subscription  # 사용되지 않는 변수 경고 방지

        self.bridge = CvBridge()
        self.latest_ros_frame = None
        self.lock_image = threading.Lock()
        self.frame_available = threading.Event()

        self.latest_detection_info = []
        self.lock_detection = threading.Lock()

        self.get_logger().info('ImageSubscriber 노드가 시작되었습니다.')

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
            self.get_logger().debug('image_raw 토픽에서 새로운 프레임을 수신했습니다.')
        except Exception as e:
            self.get_logger().error(f'image_raw 이미지 변환 오류: {e}')

    def listener_callback_detection(self, msg):
        try:
            # detection_info의 JSON 데이터를 파싱
            detection_info = json.loads(msg.data)
            detections = detection_info.get('detections', [])
            box_coordinates = [det['box_coordinates'] for det in detections]

            with self.lock_detection:
                self.latest_detection_info = box_coordinates

            self.get_logger().debug(f'detection_info 토픽에서 {len(box_coordinates)}개의 박스를 수신했습니다.')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'detection_info JSON 디코딩 오류: {e}')
        except Exception as e:
            self.get_logger().error(f'detection_info 처리 중 오류: {e}')

    def display_images(self):
        while rclpy.ok():
            # 새로운 프레임이 도착할 때까지 대기
            if self.frame_available.wait(timeout=1.0):
                with self.lock_image:
                    frame = self.latest_ros_frame.copy() if self.latest_ros_frame is not None else None
                self.frame_available.clear()

                if frame is not None:
                    try:
                        # 프레임 크기 조정 (디스플레이 속도 향상을 위해)
                        frame_display = cv2.resize(frame, (640, 480))  # 필요에 따라 해상도 조정

                        # 최신 detection_info 가져오기
                        with self.lock_detection:
                            detections = self.latest_detection_info.copy()

                        # 각 박스의 중앙 좌표를 계산하고 표시
                        for box in detections:
                            if len(box) != 4:
                                continue  # 유효하지 않은 박스 좌표 무시
                            x1, y1, x2, y2 = box
                            center_x = int((x1 + x2) / 2)
                            center_y = int((y1 + y2) / 2)
                            center_text = f'({center_x}, {center_y})'

                            # 중앙에 작은 원 그리기
                            cv2.circle(frame_display, (center_x, center_y), 5, (0, 255, 0), -1)

                            # 중앙 좌표 텍스트 표시
                            cv2.putText(frame_display, center_text, 
                                        (center_x + 10, center_y - 10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 
                                        0.5, (255, 255, 255), 2)

                        # 이미지 창에 표시
                        cv2.imshow('Received Image', frame_display)
                        # 키 입력 대기 (1ms)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            self.get_logger().info('이미지 표시를 종료합니다.')
                            rclpy.shutdown()
                            break
                    except Exception as e:
                        self.get_logger().error(f'이미지 표시 중 오류: {e}')
            else:
                self.get_logger().warning('프레임 수신 대기 중 타임아웃 발생.')

    def destroy_node(self):
        self.get_logger().info('ImageSubscriber 노드를 종료합니다.')
        cv2.destroyAllWindows()
        super().destroy_node()

# 메인 함수
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