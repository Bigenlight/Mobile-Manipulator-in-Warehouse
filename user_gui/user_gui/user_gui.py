import sys
import threading
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QSpinBox, QHBoxLayout
from PyQt5.QtCore import Qt

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool  # Import Bool for /belt topic

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class SimpleOrderGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node  # ROS2 node
        self.init_ui()

        # Publisher for /order topic
        qos_order = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_ALL,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.order_publisher = self.node.create_publisher(String, '/order', qos_profile=qos_order)

        # Publisher for /belt topic
        qos_belt = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        self.belt_publisher = self.node.create_publisher(Bool, '/belt', qos_profile=qos_belt)

    def init_ui(self):
        # Create main layout
        main_layout = QVBoxLayout()
        main_layout.setAlignment(Qt.AlignCenter)
        main_layout.setSpacing(20)

        # Red box layout
        red_layout = QVBoxLayout()
        red_label = QLabel('red box')
        red_label.setAlignment(Qt.AlignCenter)
        self.red_spinbox = QSpinBox()
        self.red_spinbox.setRange(0, 5)
        self.red_spinbox.setValue(0)
        red_layout.addWidget(red_label)
        red_layout.addWidget(self.red_spinbox)

        # Blue box layout
        blue_layout = QVBoxLayout()
        blue_label = QLabel('blue box')
        blue_label.setAlignment(Qt.AlignCenter)
        self.blue_spinbox = QSpinBox()
        self.blue_spinbox.setRange(0, 5)
        self.blue_spinbox.setValue(0)
        blue_layout.addWidget(blue_label)
        blue_layout.addWidget(self.blue_spinbox)

        # Goto goal layout
        goto_layout = QVBoxLayout()
        goto_label = QLabel('goto goal')
        goto_label.setAlignment(Qt.AlignCenter)
        self.goto_spinbox = QSpinBox()
        self.goto_spinbox.setRange(1, 3)
        self.goto_spinbox.setValue(1)
        goto_layout.addWidget(goto_label)
        goto_layout.addWidget(self.goto_spinbox)

        # Send button
        self.send_button = QPushButton('send')
        self.send_button.clicked.connect(self.send_order)
        self.send_button.setFixedSize(100, 40)

        # Belt control buttons layout
        belt_layout = QHBoxLayout()
        belt_layout.setSpacing(20)

        # On button
        self.on_button = QPushButton('on')
        self.on_button.clicked.connect(self.belt_on)
        self.on_button.setFixedSize(80, 40)
        self.on_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border-radius: 10px;
                font-size: 16px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3e8e41;
            }
        """)

        # Off button
        self.off_button = QPushButton('off')
        self.off_button.clicked.connect(self.belt_off)
        self.off_button.setFixedSize(80, 40)
        self.off_button.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                border-radius: 10px;
                font-size: 16px;
            }
            QPushButton:hover {
                background-color: #da190b;
            }
            QPushButton:pressed {
                background-color: #b71c1c;
            }
        """)

        belt_layout.addWidget(self.on_button)
        belt_layout.addWidget(self.off_button)

        # Add layouts to main layout
        main_layout.addLayout(red_layout)
        main_layout.addLayout(blue_layout)
        main_layout.addLayout(goto_layout)
        main_layout.addWidget(self.send_button, alignment=Qt.AlignCenter)
        main_layout.addLayout(belt_layout)  # Add belt buttons below send button

        self.setLayout(main_layout)
        self.setWindowTitle('Order GUI')
        self.setFixedSize(300, 500)  # Increased height to accommodate new buttons
        self.show()

    def send_order(self):
        red_value = self.red_spinbox.value()
        blue_value = self.blue_spinbox.value()
        goto_value = self.goto_spinbox.value()
        order_str = f'red_box:{red_value},blue_box:{blue_value},goto_goal:{goto_value}'
        msg = String()
        msg.data = order_str
        self.order_publisher.publish(msg)
        self.node.get_logger().info(f'Published to /order: {order_str}')

    def belt_on(self):
        msg = Bool()
        msg.data = True
        self.belt_publisher.publish(msg)
        self.node.get_logger().info('Published to /belt: True')

    def belt_off(self):
        msg = Bool()
        msg.data = False
        self.belt_publisher.publish(msg)
        self.node.get_logger().info('Published to /belt: False')

def main(args=None):
    rclpy.init(args=args)
    node = Node('simple_order_gui_node')

    app = QApplication(sys.argv)
    gui = SimpleOrderGUI(node)

    # Start ROS2 spinning in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    exit_code = app.exec_()

    rclpy.shutdown()
    ros_thread.join()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
