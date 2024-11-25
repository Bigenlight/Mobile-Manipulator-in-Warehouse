import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
import serial
import time

class OrderBeltControl(Node):
    def __init__(self):
        super().__init__('order_belt_control')

        # Serial port settings
        self.serial_port = '/dev/arduino_trailer'  # Update this to match your environment
        self.baud_rate = 115200

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            self.get_logger().info(f'Connected to serial port {self.serial_port}.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            self.ser = None

        # Subscribe to /belt topic
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

        # Subscribe to 'state' topic
        self.state_subscription = self.create_subscription(
            Int32,
            'state',
            self.state_callback,
            10  # Use default QoS
        )

        # Error publisher
        self.error_publisher = self.create_publisher(String, '/error', 10)

        self.get_logger().info('OrderBeltControl Node has been started.')

        # Last successful communication time
        self.last_successful_communication_time = self.get_clock().now()

        # Error published flag
        self.error_published = False

        # Timer to check serial connection every 1 second
        self.serial_check_timer = self.create_timer(1.0, self.check_serial_connection)

        # Timer to read serial data every 0.1 seconds
        self.serial_read_timer = self.create_timer(0.1, self.read_serial_data)

    def belt_callback(self, msg):
        if msg.data:
            # If belt is ON, send 10000 steps
            self.send_command_to_belt('10000')
        else:
            # If belt is OFF, send 'STOP' to stop the Arduino
            self.send_command_to_belt('STOP')

    def state_callback(self, msg):
        if msg.data == 4:
            # Send 1050 steps
            self.send_command_to_belt('1050')
        elif msg.data == 5:
            # Send 10000 steps
            self.send_command_to_belt('10000')

    def send_command_to_belt(self, command_str):
        if self.ser and self.ser.is_open:
            try:
                command = f'{command_str}\n'
                self.ser.write(command.encode('utf-8'))
                self.get_logger().info(f'Sent command to belt: {command_str}')
                # Update last successful communication time
                self.last_successful_communication_time = self.get_clock().now()
                # Reset error_published flag if needed
                if self.error_published:
                    self.error_published = False
                    self.get_logger().info('Arduino connection restored.')
            except serial.SerialException as e:
                self.get_logger().warn(f'Serial port write error: {e}')
        else:
            self.get_logger().warn('Serial port is not open.')

    def read_serial_data(self):
        if self.ser and self.ser.is_open:
            try:
                while self.ser.in_waiting > 0:
                    data = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if data:
                        self.get_logger().info(f'Received from Arduino: {data}')
                        # Update last successful communication time
                        self.last_successful_communication_time = self.get_clock().now()
                        # Reset error_published flag if needed
                        if self.error_published:
                            self.error_published = False
                            self.get_logger().info('Arduino connection restored.')
            except serial.SerialException as e:
                self.get_logger().warn(f'Serial port read error: {e}')
        else:
            self.get_logger().warn('Serial port is not open.')

    def check_serial_connection(self):
        now = self.get_clock().now()
        delta = now - self.last_successful_communication_time
        delta_seconds = delta.nanoseconds / 1e9  # Convert nanoseconds to seconds
        if delta_seconds > 20.0:
            if not self.error_published:
                # Publish error message
                error_msg = String()
                error_msg.data = 'Arduino has no response'
                self.error_publisher.publish(error_msg)
                self.get_logger().error('Arduino has no response')
                self.error_published = True

    def destroy_node(self):
        # Cancel timers
        if self.serial_read_timer:
            self.serial_read_timer.cancel()
        if self.serial_check_timer:
            self.serial_check_timer.cancel()
        # Close serial port
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
