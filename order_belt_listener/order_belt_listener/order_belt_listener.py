# order_belt_listener.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class OrderBeltListener(Node):
    def __init__(self):
        super().__init__('order_belt_listener')
        
        # QoS Profiles 설정
        qos_belt = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )
        
        # /belt 토픽 구독
        self.belt_subscription = self.create_subscription(
            Bool,
            '/belt',
            self.belt_callback,
            qos_belt
        )
        self.belt_subscription  # Prevent unused variable warning
        
        self.get_logger().info('OrderBeltListener Node has been started.')

    def belt_callback(self, msg):
        if msg.data:
            self.get_logger().info('컨베이어가 on되었습니다.')
        else:
            self.get_logger().info('컨베이어가 off되었습니다.')

def main(args=None):
    rclpy.init(args=args)
    order_belt_listener = OrderBeltListener()
    try:
        rclpy.spin(order_belt_listener)
    except KeyboardInterrupt:
        pass
    finally:
        order_belt_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
