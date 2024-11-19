# order_listener_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class OrderListener(Node):
    def __init__(self):
        super().__init__('order_listener')
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_ALL,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.subscription = self.create_subscription(
            String,
            '/order',
            self.listener_callback,
            qos_profile
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Order Listener Node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received raw order: {msg.data}')
        try:
            # 문자열을 쉼표로 분리
            parts = msg.data.split(',')
            order_dict = {}
            for part in parts:
                key, value = part.split(':')
                order_dict[key.strip()] = value.strip()
            
            # 각 값 추출
            red_box = order_dict.get('red_box', '0')
            blue_box = order_dict.get('blue_box', '0')
            goto_goal = order_dict.get('goto_goal', '0')
            
            # 개별 값 출력
            self.get_logger().info(f'Red Box: {red_box}')
            self.get_logger().info(f'Blue Box: {blue_box}')
            self.get_logger().info(f'Goto Goal: {goto_goal}')
        except Exception as e:
            self.get_logger().error(f'Failed to parse message: {e}')

def main(args=None):
    rclpy.init(args=args)
    order_listener = OrderListener()
    try:
        rclpy.spin(order_listener)
    except KeyboardInterrupt:
        pass
    finally:
        order_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
