# state_publisher.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time

class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')
        self.publisher_ = self.create_publisher(Int32, 'state', 10)
        self.timer_period = 4  # 3초마다 퍼블리시
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.state = 0
        self.repeat_count = 0  # 4번 상태를 퍼블리시한 횟수
        self.get_logger().info('State Publisher가 시작되었습니다.')

    def timer_callback(self):
        # 4번 상태만 3번 퍼블리시
        if self.state == 4:
            if self.repeat_count < 3:
                msg = Int32()
                msg.data = self.state
                self.publisher_.publish(msg)
                self.get_logger().info(f'퍼블리시된 상태: {self.state}')
                self.repeat_count += 1
            else:
                # 3번 퍼블리시 후 다음 상태로 넘어감
                self.state += 1
                self.repeat_count = 0
        else:
            msg = Int32()
            msg.data = self.state
            self.publisher_.publish(msg)
            self.get_logger().info(f'퍼블리시된 상태: {self.state}')
            self.state += 1

        # 상태 초기화
        if self.state > 10:
            self.state = 0

def main(args=None):
    rclpy.init(args=args)
    state_publisher = StatePublisher()
    try:
        rclpy.spin(state_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        state_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
