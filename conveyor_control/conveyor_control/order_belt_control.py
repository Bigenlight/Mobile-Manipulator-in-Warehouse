# conveyor_control/order_belt_control.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial
import time

class OrderBeltControl(Node):
    def __init__(self):
        super().__init__('order_belt_control')

        # 시리얼 포트 설정 (포트명과 보드레이트는 환경에 맞게 수정)
        self.serial_port = '/dev/ttyUSB0'  # 예: /dev/ttyACM0, Windows의 경우 'COM3' 등
        self.baud_rate = 115200

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)  # 아두이노 리셋 대기
            self.get_logger().info(f'시리얼 포트 {self.serial_port}에 연결되었습니다.')
        except serial.SerialException as e:
            self.get_logger().error(f'시리얼 포트 연결 실패: {e}')
            self.ser = None

        # /belt 토픽 구독
        qos_belt = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )

        self.belt_subscription = self.create_subscription(
            Bool,
            '/belt',
            self.belt_callback,
            qos_belt
        )

        self.belt_subscription  # Prevent unused variable warning

        self.get_logger().info('OrderBeltControl Node has been started.')

    def belt_callback(self, msg):
        if self.ser and self.ser.is_open:
            if msg.data:
                command = 'ON\n'
                self.ser.write(command.encode('utf-8'))
                self.get_logger().info('컨베이어를 작동시켰습니다.')
            else:
                command = 'OFF\n'
                self.ser.write(command.encode('utf-8'))
                self.get_logger().info('컨베이어를 정지시켰습니다.')
        else:
            self.get_logger().warn('시리얼 포트가 열려 있지 않습니다.')

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    order_belt_control = OrderBeltControl()
    try:
        rclpy.spin(order_belt_control)
    except KeyboardInterrupt:
        pass
    finally:
        order_belt_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
