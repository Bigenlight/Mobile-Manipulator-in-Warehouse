import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import threading
import time
import os
import math
import argparse
import json
from datetime import datetime

# Import DeepSORT
from deep_sort_realtime.deepsort_tracker import DeepSort

class YoloPublisher(Node):
    def __init__(self, model_path):
        super().__init__('yolo_publisher')
        
        # Publishers
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.alarm_publisher = self.create_publisher(String, 'alarm', 10)
        self.detection_publisher = self.create_publisher(String, 'detection_info', 10)
        
        # Bridge and Model
        self.bridge = CvBridge()
        self.model = YOLO(model_path)
        
        # Initialize DeepSORT Tracker
        self.tracker = DeepSort(
            max_age=30,
            n_init=3,
            nms_max_overlap=1.0,
            max_cosine_distance=0.2,
            nn_budget=None,
            override_track_class=None,
            embedder="mobilenet",
            half=True,
            bgr=True,
            embedder_gpu=True,
            embedder_model_name="mars-small128.pb",
            embedder_warmup=0,
        )

        # Initialization
        self.output_dir = './output'
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Video capture setup
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Width
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Height
        
        # Get image dimensions
        self.image_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.image_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # Control flags
        self.frame_count = 0  # Used to process every other frame
        self.lock = threading.Lock()
        self.current_frame = None
        self.processed_frame = None
        self.classNames = ['blue', 'red']
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
        """Process frames for object detection and tracking in a separate thread."""
        while rclpy.ok():
            if self.current_frame is not None and self.frame_count % 2 == 0:
                with self.lock:
                    frame_to_process = self.current_frame.copy()
                    
                blue_detected = False  # Flag to check if a blue object is detected
                detection_info_list = []  # List to store detection info
                blue_count = 0  # Initialize blue count
                red_count = 0  # Initialize red count

                # Run YOLO object detection
                results = self.model(frame_to_process, stream=True)
                
                # Prepare detections for DeepSORT
                detections = []
                detection_classes = []
                for r in results:
                    for box in r.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        confidence = float(box.conf[0])
                        cls = int(box.cls[0])
                        if cls >= len(self.classNames):
                            continue  # Skip unknown classes
                        detections.append(([x1, y1, x2 - x1, y2 - y1], confidence, cls))
                        detection_classes.append(cls)

                # Update tracker with detections
                tracks = self.tracker.update_tracks(detections, frame=frame_to_process)

                for track in tracks:
                    if not track.is_confirmed():
                        continue
                    track_id = track.track_id
                    ltrb = track.to_ltrb()
                    x1, y1, x2, y2 = map(int, ltrb)
                    cls = track.det_class  # Detected class
                    confidence = track.det_conf

                    # Draw bounding box and put text including the ID
                    cv2.rectangle(frame_to_process, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green bounding box
                    cv2.putText(
                        frame_to_process,
                        f"ID: {track_id}, {self.classNames[cls]}: {confidence:.2f}",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )

                    # Prepare detection info
                    detection_info = {
                        'id': track_id,
                        'time': datetime.now().isoformat(),
                        'box_coordinates': [x1, y1, x2, y2],
                        'class_number': cls,
                        'confidence': confidence
                    }
                    detection_info_list.append(detection_info)

                    # Update counts based on class
                    if cls == 0:  # blue
                        blue_detected = True
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
    
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='YoloPublisher Node')
    parser.add_argument('--user', type=str, default='rokey', help='User name for model path')
    args, unknown = parser.parse_known_args()
    
    # Model path
    model_path = f'/home/{args.user}/3_ws/src/best.pt'
    
    # Create and spin the node
    node = YoloPublisher(model_path)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
