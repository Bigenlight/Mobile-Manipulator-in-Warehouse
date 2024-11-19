import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        
        # Create a publisher for the 'image_topic'
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)
        
        # Set timer to publish image every 5 seconds
        self.timer = self.create_timer(5.0, self.publish_image)
        
        self.bridge = CvBridge()
        
        self.get_logger().info('Image Publisher Node has been started.')
        
        # List of image filenames to publish
        self.images = ['0_img_29.jpg', '1_img_17.jpg']
        
        # Initialize image index
        self.current_image = 0
        
        # Determine the directory where the script is located
        self.script_dir = os.path.dirname(os.path.realpath(__file__))
        
        # Define the relative path to the images directory
        self.images_dir = os.path.join(self.script_dir, 'images')
        
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
            # Convert the image to ROS2 Image message
            msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            
            # Publish the message
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published image: {image_filename}')
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
