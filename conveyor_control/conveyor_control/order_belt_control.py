import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
import serial
import time

class OrderBeltControl(Node):
    def __init__(self):
        super().__init__('order_belt_control')

        # 시리얼 포트 설정 (포트명과 보드레이트는 환경에 맞게 수정)
        self.serial_port = '/dev/arduino_trailer'  # 예: /dev/ttyACM0, Windows의 경우 'COM3' 등
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

        # 'state' 토픽 구독 추가
        self.state_subscription = self.create_subscription(
            Int32,
            'state',
            self.state_callback,
            10  # Use default QoS
        )

        self.state_subscription  # Prevent unused variable warning

        self.get_logger().info('OrderBeltControl Node has been started.')

        # 타이머를 저장하기 위한 변수
        self.state_timer = None

        # 컨베이어 벨트 상태 관리
        self.belt_sources = set()

    def belt_callback(self, msg):
        if msg.data:
            if 'belt' not in self.belt_sources:
                self.belt_sources.add('belt')
                if len(self.belt_sources) == 1:
                    # Belt was previously off, need to turn it on
                    self.send_command_to_belt('ON')
        else:
            if 'belt' in self.belt_sources:
                self.belt_sources.remove('belt')
                if len(self.belt_sources) == 0:
                    # No more sources, turn off the belt
                    self.send_command_to_belt('OFF')

    def state_callback(self, msg):
        if msg.data == 4:
            self.handle_state_on_off(2)
        elif msg.data == 5:
            self.handle_state_on_off(15)

    def handle_state_on_off(self, delay_seconds):
        if 'state' not in self.belt_sources:
            self.belt_sources.add('state')
            if len(self.belt_sources) == 1:
                # Belt was previously off, need to turn it on
                self.send_command_to_belt('ON')
            self.get_logger().info(f'컨베이어를 작동시켰습니다. {delay_seconds}초 후에 정지합니다.')
        else:
            self.get_logger().info(f'컨베이어 상태 유지. {delay_seconds}초 후에 상태를 업데이트합니다.')

        # Start or reset the timer for 'state'
        if self.state_timer:
            self.state_timer.cancel()
        self.state_timer = self.create_timer(
            delay_seconds,
            self.state_timer_callback
        )

    def state_timer_callback(self):
        self.get_logger().info('state_timer_callback called')
        if 'state' in self.belt_sources:
            self.belt_sources.remove('state')
            if len(self.belt_sources) == 0:
                # No more sources, turn off the belt
                self.send_command_to_belt('OFF')
        # Do NOT set self.state_timer = None here

    def send_command_to_belt(self, command_str):
        if self.ser and self.ser.is_open:
            command = f'{command_str}\n'
            self.ser.write(command.encode('utf-8'))
            self.get_logger().info(f'컨베이어를 {command_str}했습니다.')
        else:
            self.get_logger().warn('시리얼 포트가 열려 있지 않습니다.')

    def destroy_node(self):
        # 노드를 파괴할 때 타이머를 취소
        if self.state_timer:
            self.state_timer.cancel()

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
