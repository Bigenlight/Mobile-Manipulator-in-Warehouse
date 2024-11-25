#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import threading
import os
import sys
from ultralytics import YOLO

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # Subscribe to the image topic
        self.image_subscription = self.create_subscription(
            Image,
            '/image_raw/compressed',  # Match the publisher's topic name
            self.listener_callback_image,
            10)
        self.image_subscription  # Prevent unused variable warning

        # Subscribe to the 'target_box' topic
        self.target_subscription = self.create_subscription(
            String,
            'target_box',
            self.target_callback,
            10)
        self.target_subscription  # Prevent unused variable warning

        # Publisher for 'pixel_coord' topic (as String)
        self.pixel_publisher = self.create_publisher(String, 'pixel_coord', 10)

        # Publisher for 'count_box' topic (as String)
        self.count_publisher = self.create_publisher(String, 'count_box', 10)

        # **Add the missing publisher for '/image_topic'**
        # Publisher for '/image_topic' (CompressedImage)
        self.image_publisher = self.create_publisher(CompressedImage, '/image_topic', 10)

        self.bridge = CvBridge()
        self.latest_ros_frame = None
        self.lock_image = threading.Lock()
        self.frame_available = threading.Event()

        # Tracking-related variables
        self.target_color = None  # 'red' or 'blue'
        self.tracking_active = False

        self.get_logger().info('ImageSubscriber node has started.')

        # Set the path to your YOLOv8 model file
        model_path = '/home/theo/3_ws/src/best.pt'  # Adjust to your model's path

        # Check if the YOLOv8 model file exists
        if not os.path.isfile(model_path):
            self.get_logger().fatal(f'Cannot find YOLOv8 model file: {model_path}')
            rclpy.shutdown()
            sys.exit(1)

        # Load the YOLOv8 model
        try:
            self.model = YOLO(model_path)
            self.get_logger().info('Successfully loaded YOLOv8 model.')
        except Exception as e:
            self.get_logger().fatal(f'Error loading YOLOv8 model: {e}')
            rclpy.shutdown()
            sys.exit(1)

        # Start the image display and processing in a separate thread
        self.display_thread = threading.Thread(target=self.display_images)
        self.display_thread.daemon = True
        self.display_thread.start()

    def listener_callback_image(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock_image:
                self.latest_ros_frame = cv_image.copy()
            self.frame_available.set()
            self.get_logger().debug('Received a new frame from /image_raw/compressed topic.')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def target_callback(self, msg):
        color = msg.data.lower().strip()
        if color in ['red', 'blue']:
            self.target_color = color
            self.tracking_active = True
            self.get_logger().info(f"Started tracking '{self.target_color}' boxes.")
            self.publish_current_image()
        elif color == 'done':
            self.tracking_active = False
            self.target_color = None  # Reset target color
            self.get_logger().info("Stopped tracking boxes.")
            self.publish_current_image()
        else:
            self.get_logger().warning(f"Received unknown command: '{msg.data}'")

    def publish_current_image(self):
        with self.lock_image:
            if self.latest_ros_frame is not None:
                # Convert OpenCV image to ROS CompressedImage message
                try:
                    compressed_msg = self.bridge.cv2_to_compressed_imgmsg(self.latest_ros_frame)
                    compressed_msg.header.stamp = self.get_clock().now().to_msg()
                    compressed_msg.format = "jpeg"
                    self.image_publisher.publish(compressed_msg)
                    self.get_logger().info("Published current image to '/image_topic'.")
                except Exception as e:
                    self.get_logger().error(f"Failed to publish image: {e}")
            else:
                self.get_logger().warning("No image available to publish.")

    def display_images(self):
        while rclpy.ok():
            if self.frame_available.wait(timeout=1.0):
                with self.lock_image:
                    frame = self.latest_ros_frame.copy() if self.latest_ros_frame is not None else None
                self.frame_available.clear()

                if frame is not None:
                    try:
                        # Use YOLOv8 tracking
                        results = self.model.track(frame, persist=True, verbose=False)

                        # Initialize counts
                        red_count = 0
                        blue_count = 0

                        # Initialize variables to select the box with the smallest Track ID
                        selected_box = None
                        min_track_id = float('inf')

                        # Process all detected boxes
                        for result in results:
                            boxes = result.boxes

                            for box in boxes:
                                # Extract box information
                                x1, y1, x2, y2 = box.xyxy[0]
                                confidence = box.conf[0]
                                class_id = int(box.cls[0])
                                class_name = self.model.names[class_id]
                                track_id = int(box.id[0]) if box.id is not None else -1

                                # Convert coordinates to integers for drawing
                                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                                confidence = float(confidence)

                                # Draw bounding box
                                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                label = f"ID: {track_id}, {class_name}, {confidence:.2f}"
                                cv2.putText(frame, label, (x1, y1 - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                                # Calculate center coordinates (float)
                                center_x = (x1 + x2) / 2.0
                                center_y = (y1 + y2) / 2.0

                                # Draw center point (convert to int for drawing)
                                center_x_int = int(center_x)
                                center_y_int = int(center_y)
                                cv2.circle(frame, (center_x_int, center_y_int), 5, (255, 0, 0), -1)
                                coord_label = f"({center_x_int}, {center_y_int})"
                                cv2.putText(frame, coord_label, (center_x_int + 10, center_y_int),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                                # Log detailed information
                                self.get_logger().info(
                                    f"Track ID: {track_id}, Class: {class_name}, Confidence: {confidence:.2f}, "
                                    f"Coordinates: ({x1}, {y1}), ({x2}, {y2}), Center: ({center_x:.1f}, {center_y:.1f})"
                                )

                                # Increment counts
                                if class_name.lower() == 'red':
                                    red_count += 1
                                elif class_name.lower() == 'blue':
                                    blue_count += 1

                                # Check if tracking is active and box matches target color
                                if self.tracking_active and self.target_color == class_name.lower():
                                    if track_id != -1 and track_id < min_track_id:
                                        min_track_id = track_id
                                        selected_box = {'center_x': center_x, 'center_y': center_y}

                        # Publish the counts
                        count_msg = String()
                        count_msg.data = f'red_count:{red_count},blue_count:{blue_count}'
                        self.count_publisher.publish(count_msg)
                        self.get_logger().info(f"Published counts: {count_msg.data}")

                        # Publish the center coordinates if tracking is active
                        if self.tracking_active:
                            if selected_box is not None:
                                coord_str = f"({selected_box['center_x']:.1f},{selected_box['center_y']:.1f})"
                                pixel_msg = String()
                                pixel_msg.data = coord_str

                                self.pixel_publisher.publish(pixel_msg)
                                self.get_logger().info(
                                    f"Published pixel coordinates: {pixel_msg.data}"
                                )
                            else:
                                self.get_logger().warning(f"Could not find a '{self.target_color}' box.")
                        # Else, do not publish pixel coordinates or log warnings

                        # Display the image
                        cv2.imshow('YOLOv8 Detection', frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            self.get_logger().info('Exiting image display.')
                            rclpy.shutdown()
                            break
                    except Exception as e:
                        self.get_logger().error(f'Error during YOLOv8 processing: {e}')
            else:
                self.get_logger().warning('Frame reception timed out.')

    def destroy_node(self):
        self.get_logger().info('Shutting down ImageSubscriber node.')
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
