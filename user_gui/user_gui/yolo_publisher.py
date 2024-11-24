import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # 스트링
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import threading
import time
import os
import math
from shapely.geometry import Polygon
import argparse  # argparse 모듈 추가
import json  # Import json for serialization
from datetime import datetime  # Import datetime for timestamp formatting

class YoloPublisher(Node):
    def __init__(self, model_path):
        super().__init__('yolo_publisher')
        
        # Publishers
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)  # 토픽 이름 변경
        self.alarm_publisher = self.create_publisher(String, 'alarm', 10)  # 알람 퍼블리시
        self.detection_publisher = self.create_publisher(String, 'detection_info', 10)  # New publisher
        
        # Bridge and Model
        self.bridge = CvBridge()
        self.model = YOLO(model_path)  # 전달받은 모델 경로 사용
        
        # Initialization
        self.coordinates = []
        self.output_dir = './output'
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Video capture setup
        self.cap = cv2.VideoCapture(2)  # Adjusted to default camera source (0)
        self.cap.set(3, 640)  # Width
        self.cap.set(4, 480)  # Height
        
        # Get image dimensions
        self.image_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.image_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # Define the rectangle in the top-right corner (1.3 times the original height)
        self.rect_x1 = int(self.image_width * 0.5)  # Starting at 50% of the width
        self.rect_y1 = 0  # Top of the image
        self.rect_x2 = self.image_width  # Right edge of the image
        self.rect_y2 = int(self.image_height * 0.65)  # 65% of the height
        
        # Control flags
        self.frame_count = 0  # Used to process every other frame
        self.lock = threading.Lock()
        self.current_frame = None
        self.processed_frame = None
        self.classNames = ['blue' , 'red']
        self.blue_previously_detected = False  # Tracks previous detection state

        # Start threads
        threading.Thread(target=self.capture_frames, daemon=True).start()
        threading.Thread(target=self.process_frames, daemon=True).start()
        self.timer = self.create_timer(0.1, self.publish_image)

    def capture_frames(self):
        """Continuously capture frames from the camera in a separate thread."""
        while rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.current_frame = frame.copy()
            time.sleep(0.01)  # Slight delay to control frame rate

    def process_frames(self):
        """Process frames for object detection in a separate thread."""
        while rclpy.ok():
            if self.current_frame is not None and self.frame_count % 2 == 0:
                with self.lock:
                    frame_to_process = self.current_frame.copy()
                    
                blue_detected = False  # Flag to check if a blue object is detected
                detection_info_list = []  # List to store detection info
                blue_count = 0  # 카운트 초기화
                red_count = 0  # 카운트 초기화

                # Draw the red rectangle in the top-right corner
                cv2.rectangle(frame_to_process, 
                              (self.rect_x1, self.rect_y1), 
                              (self.rect_x2, self.rect_y2), 
                              (0, 0, 255), 4)  # Red rectangle with thickness 4

                # Define the rectangle polygon
                rect_polygon = Polygon([
                    (self.rect_x1, self.rect_y1),
                    (self.rect_x2, self.rect_y1),
                    (self.rect_x2, self.rect_y2),
                    (self.rect_x1, self.rect_y2)
                ])

                # Run YOLO object detection on every other frame
                results = self.model(frame_to_process, stream=True)
                for r in results:
                    for box in r.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        confidence = math.ceil(box.conf[0] * 100) / 100
                        cls = int(box.cls[0])
                        # 감시 박스
                        cv2.rectangle(frame_to_process, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Red bounding box
                        cv2.putText(frame_to_process, f"{self.classNames[cls]}: {confidence}", (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)  # Blue text above the box
                        
                        # Create a polygon for the detection box
                        box_polygon = Polygon([
                            (x1, y1),
                            (x2, y1),
                            (x2, y2),
                            (x1, y2)
                        ])
                        # Check if the detection box intersects with the rectangle
                        
                        # Get the current time for each detection
                        current_time_ros = self.get_clock().now()
                        current_time_nanosec = current_time_ros.nanoseconds
                        current_time = datetime.fromtimestamp(current_time_nanosec / 1e9).isoformat()
                        detection_info = {
                            'time': current_time,
                            'box_coordinates': [x1, y1, x2, y2],
                            'class_number': cls,
                            'confidence': confidence
                        }
                        detection_info_list.append(detection_info)
                        # 클래스에 따라 카운트 증가 및 플래그 설정
                        if cls == 0:  # blue
                            blue_detected = True  # Set flag since a blue object is detected within the rectangle
                            blue_count += 1
                        elif cls == 1:  # red
                            red_count += 1

                # Publish the alarm message based on blue detection
                if blue_detected and not self.blue_previously_detected:
                    # Publish alarm message
                    alarm_msg = String()
                    alarm_msg.data = 'blue detected'
                    self.alarm_publisher.publish(alarm_msg)

                elif not blue_detected and self.blue_previously_detected:
                    # Publish alarm message when blue object is no longer detected
                    alarm_msg = String()
                    alarm_msg.data = 'blue no longer detected'
                    self.alarm_publisher.publish(alarm_msg)

                # Prepare the detection info with counts
                detection_info_msg = {
                    'counts': {
                        'blue': blue_count,
                        'red': red_count
                    },
                    'detections': detection_info_list
                }
                
                # Publish the detection info
                detection_info_publisher = String()
                detection_info_publisher.data = json.dumps(detection_info_msg)
                self.detection_publisher.publish(detection_info_publisher)

                # Update the previous detection state
                self.blue_previously_detected = blue_detected

                # Update processed frame
                with self.lock:
                    self.processed_frame = frame_to_process

            self.frame_count += 1
            time.sleep(0.01)  # Adjust delay if necessary to control processing rate

    def publish_image(self):
        """Publish the latest processed frame."""
        if self.processed_frame is not None:
            with self.lock:
                ros_image = self.bridge.cv2_to_imgmsg(self.processed_frame, encoding="bgr8")
            self.publisher_.publish(ros_image)
            self.get_logger().debug('Published processed image.')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    # argparse를 사용하여 명령줄 인자 파싱
    parser = argparse.ArgumentParser(description='YoloPublisher Node')
    parser.add_argument('--user', type=str, default='rokey', help='User name for model path')
    args, unknown = parser.parse_known_args()
    
    # 모델 경로 생성
    model_path = f'/home/{args.user}/3_ws/src/best.pt'
    
    # YoloPublisher 노드 생성 시 모델 경로 전달
    node = YoloPublisher(model_path)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
