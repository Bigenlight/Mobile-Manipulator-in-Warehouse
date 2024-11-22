# image_subscriber.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import numpy as np
import time

# ROS ImageSubscriber Node
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # 이미지 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            'image_raw',  # 퍼블리셔의 토픽 이름과 일치시킴
            self.listener_callback1,
            10)
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()
        self.latest_ros_frame1 = None
        self.lock_ros1 = threading.Lock()
        self.frame_available_ros1 = threading.Event()
        
        self.get_logger().info('ImageSubscriber node has been started.')

        # 별도의 스레드에서 이미지 표시
        self.display_thread = threading.Thread(target=self.display_images)
        self.display_thread.daemon = True
        self.display_thread.start()

    def listener_callback1(self, msg):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock_ros1:
                self.latest_ros_frame1 = cv_image.copy()
            self.frame_available_ros1.set()
            self.get_logger().debug('Received new frame on image_raw')
        except Exception as e:
            self.get_logger().error(f'Error converting image from image_raw: {e}')

    def display_images(self):
        while rclpy.ok():
            # 새로운 프레임이 도착할 때까지 대기
            if self.frame_available_ros1.wait(timeout=1.0):
                with self.lock_ros1:
                    frame = self.latest_ros_frame1.copy() if self.latest_ros_frame1 is not None else None
                    self.frame_available_ros1.clear()
                if frame is not None:
                    try:
                        # 프레임 크기 조정 (인코딩 시간 단축)
                        frame = cv2.resize(frame, (480, 360))  # 필요에 따라 해상도 조정
                        # 창에 이미지 표시
                        cv2.imshow('Received Image', frame)
                        # 창이 업데이트되도록 기다림
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            self.get_logger().info('Quitting image display.')
                            rclpy.shutdown()
                            break
                    except Exception as e:
                        self.get_logger().error(f'Error displaying image: {e}')
            else:
                self.get_logger().warning('No new frame received within timeout.')

# 메인 함수
def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
