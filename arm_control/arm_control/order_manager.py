# arm_control/order_manager.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

class OrderManager(Node):
    def __init__(self):
        super().__init__('order_manager')
        # Publishers
        self.target_box_pub = self.create_publisher(String, '/target_box', 10)
        self.state_pub = self.create_publisher(Int32, '/state', 10)
        # Subscribers
        self.order_sub = self.create_subscription(String, '/order', self.order_callback, 10)
        self.load_on_conveyor_sub = self.create_subscription(String, '/load_on_conveyor', self.load_on_conveyor_callback, 10)
        # Internal variables
        self.current_order = []
        self.waiting_for_load_done = False

        # Define job mappings
        self.job_mappings = {
            'Job1': {'red': 2, 'blue': 1},
            'Job2': {'red': 1, 'blue': 2},
            'Job3': {'red': 1, 'blue': 0}
        }

    def order_callback(self, msg):
        self.get_logger().info(f"Received order: {msg.data}")
        order_str = msg.data.strip()

        # Reset current order
        self.current_order = []
        red_box_count = 0
        blue_box_count = 0

        # Check if the order is a predefined job
        if order_str in self.job_mappings:
            job = order_str
            red_box_count = self.job_mappings[job].get('red', 0)
            blue_box_count = self.job_mappings[job].get('blue', 0)
            self.get_logger().info(f"Parsed {job}: red={red_box_count}, blue={blue_box_count}")
        else:
            # Assume the order is in key-value pair format
            try:
                orders = order_str.split(',')
                for item in orders:
                    key, value = item.split(':')
                    key = key.strip().lower()
                    value = int(value.strip())
                    if key == 'red_box':
                        red_box_count = value
                    elif key == 'blue_box':
                        blue_box_count = value
                    elif key == 'goto_goal':
                        # Handle 'goto_goal' if needed
                        pass
                    else:
                        self.get_logger().warn(f"Unknown key '{key}' in order.")
            except Exception as e:
                self.get_logger().error(f"Failed to parse order: {e}")
                return

        # Prepare the current order list: reds first, then blues
        self.current_order = ['red'] * red_box_count + ['blue'] * blue_box_count
        self.get_logger().info(f"Current order list: {self.current_order}")
        # Overwrite any previous processing
        self.waiting_for_load_done = False
        # Start processing the new order
        self.process_next_order()

    def process_next_order(self):
        if self.current_order:
            # Get the next box color to send
            next_box = self.current_order.pop(0)
            # Publish to '/target_box'
            msg = String()
            msg.data = next_box
            self.target_box_pub.publish(msg)
            self.get_logger().info(f"Published to /target_box: {msg.data}")
            # Set flag to wait for 'done'
            self.waiting_for_load_done = True
        else:
            # All orders processed, publish state
            state_msg = Int32()
            state_msg.data = 5
            self.state_pub.publish(state_msg)
            self.get_logger().info("Published /state: 5")

    def load_on_conveyor_callback(self, msg):
        self.get_logger().info(f"Received /load_on_conveyor: {msg.data}")
        if msg.data.strip().lower() == 'done' and self.waiting_for_load_done:
            self.waiting_for_load_done = False
            # Process the next order
            self.process_next_order()

def main(args=None):
    rclpy.init(args=args)
    order_manager = OrderManager()
    rclpy.spin(order_manager)
    order_manager.destroy_node()
    rclpy.shutdown()
