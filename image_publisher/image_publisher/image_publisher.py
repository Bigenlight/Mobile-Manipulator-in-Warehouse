# image_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import os

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        
        # Create a publisher for the 'image_topic' with CompressedImage messages
        self.publisher_ = self.create_publisher(CompressedImage, 'image_topic', 10)
        
        # Set timer to publish image every 5 seconds
        self.timer = self.create_timer(5.0, self.publish_image)
        
        self.bridge = CvBridge()
        
        self.get_logger().info('Image Publisher Node has been started.')
        
        # List of image filenames to publish
        self.images = ['0_img_29.jpg', '1_img_17.jpg']
        
        # Initialize image index
        self.current_image = 0
        
        # Get the path to the images directory in the share directory
        share_dir = get_package_share_directory('image_publisher')
        self.images_dir = os.path.join(share_dir, 'images')
        
        # Verify that the images directory exists
        if not os.path.exists(self.images_dir):
            self.get_logger().error(f"Images directory does not exist: {self.images_dir}")
            raise FileNotFoundError(f"Images directory does not exist: {self.images_dir}")
        
        # Verify that all images exist
        missing_images = [img for img in self.images if not os.path.isfile(os.path.join(self.images_dir, img))]
        if missing_images:
            for img in missing_images:
                self.get_logger().error(f"Image file not found: {os.path.join(self.images_dir, img)}")
            raise FileNotFoundError(f"One or more image files are missing in {self.images_dir}")

    def publish_image(self):
        # Get the current image filename
        image_filename = self.images[self.current_image]
        
        # Construct the full image path
        image_path = os.path.join(self.images_dir, image_filename)
        
        # Read the image using OpenCV
        img = cv2.imread(image_path)
        
        if img is not None:
            # Encode image as JPEG
            success, encoded_image = cv2.imencode('.jpg', img)
            if success:
                # Create CompressedImage message
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.format = "jpeg"
                msg.data = encoded_image.tobytes()
                
                # Publish the message
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published image: {image_filename}')
            else:
                self.get_logger().warn(f'Failed to encode image: {image_path}')
        else:
            self.get_logger().warn(f'Failed to load image: {image_path}')
        
        # Update the image index for the next publish
        self.current_image = (self.current_image + 1) % len(self.images)

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Image Publisher Node has been stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
