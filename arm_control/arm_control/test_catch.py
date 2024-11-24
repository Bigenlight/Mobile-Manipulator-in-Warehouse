#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class CatchPublisher(Node):
    def __init__(self):
        super().__init__('catch_publisher')
        self.publisher_ = self.create_publisher(Empty, 'catch', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)  # Publish every 2 seconds

    def timer_callback(self):
        msg = Empty()
        self.publisher_.publish(msg)
        self.get_logger().info('Published to catch topic.')

def main(args=None):
    rclpy.init(args=args)
    node = CatchPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
