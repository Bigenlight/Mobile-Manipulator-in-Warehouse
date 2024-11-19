# combined_gui.py

import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QLineEdit, QPushButton, QVBoxLayout, QHBoxLayout,
    QSpinBox, QGroupBox, QMessageBox, QSizePolicy, QProgressBar
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer, QObject, pyqtSlot, QThread
from PyQt5.QtGui import QImage, QPixmap, QFont

from std_msgs.msg import String, Bool, Int32
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import cv2
import numpy as np
import time
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
import os
from dotenv import load_dotenv  # Python용 dotenv 가져오기
import re  # Added for email validation

# --- Communicator class for signal passing ---
class Communicator(QObject):
    error_signal = pyqtSignal(str)  # Signal to pass error messages
    state_signal = pyqtSignal(int)  # Signal to pass state updates

# --- VideoThread for video streaming ---
class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(QImage)

    def __init__(self, source=0, parent=None):
        super().__init__(parent)
        self.source = source
        self._run_flag = True

    def run(self):
        # Open video source
        cap = cv2.VideoCapture(self.source)
        if not cap.isOpened():
            self.change_pixmap_signal.emit(QImage())  # Emit empty image on failure
            self.parent().get_logger().error(f"Cannot open camera with index {self.source}")
            return
        while self._run_flag:
            ret, cv_img = cap.read()
            if ret:
                # Convert to RGB
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                height, width, channel = cv_img.shape
                bytes_per_line = 3 * width
                qt_img = QImage(cv_img.data, width, height, bytes_per_line, QImage.Format_RGB888)
                # Emit signal
                self.change_pixmap_signal.emit(qt_img)
            else:
                self.parent().get_logger().warning(f"Failed to read frame from camera {self.source}")
        # Release video source
        cap.release()

    def stop(self):
        self._run_flag = False
        self.wait()

# --- ROS2 Nodes ---
class ManipulationPublisherNode(Node):
    def __init__(self):
        super().__init__('manipulation_publisher')
        self.publisher = self.create_publisher(String, '/manipulation', 10)
        self.get_logger().info('ManipulationPublisherNode has been initialized and publishing to "/manipulation" topic.')

    def publish_command(self, command: str):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f'Published "{command}" to /manipulation topic.')

class ManipulationListenerNode(Node):
    def __init__(self):
        super().__init__('manipulation_listener')
        self.subscription = self.create_subscription(
            String,
            '/manipulation',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info("ManipulationListener node has been initialized and subscribed to /manipulation topic.")

    def listener_callback(self, msg):
        message = msg.data
        self.get_logger().info(f"Received message: {message}")

        # Perform actions based on message
        if message == "play":
            self.get_logger().info("Play command received. Executing 'play' action.")
            # Add actual action code here
        elif message == "stop":
            self.get_logger().info("Stop command received. Executing 'stop' action.")
            # Add actual action code here
        elif message == "pause":
            self.get_logger().info("Pause command received. Executing 'pause' action.")
            # Add actual action code here
        elif message == "resume":
            self.get_logger().info("Resume command received. Executing 'resume' action.")
            # Add actual action code here
        elif message == "reset":
            self.get_logger().info("Reset command received. Executing 'reset' action.")
            # Add actual action code here
        elif message == "conveyor_on":
            self.get_logger().info("Conveyor ON command received. Turning the conveyor ON.")
            # Add actual action code here
        elif message == "conveyor_off":
            self.get_logger().info("Conveyor OFF command received. Turning the conveyor OFF.")
            # Add actual action code here
        else:
            self.get_logger().warning(f"Unknown command received: {message}")

class ErrorSubscriberNode(Node):
    def __init__(self, email_config, communicator: Communicator):
        super().__init__('error_subscriber')
        self.email_config = email_config
        self.communicator = communicator
        self.subscription = self.create_subscription(
            String,
            '/error',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('ErrorSubscriberNode has been initialized and subscribed to "/error" topic.')

    def listener_callback(self, msg):
        error_message = msg.data
        self.get_logger().error(f'Received error message: {error_message}')

        # Check if receiver_email is valid
        if not self.validate_email(self.email_config['receiver_email']):
            # Set to default email
            default_email = 'kwilee0426@gmail.com'
            self.email_config['receiver_email'] = default_email
            # Emit a signal to inform GUI
            self.communicator.error_signal.emit(f"Invalid or no receiver email. Email will be sent to {default_email}.")

        # Send email in a separate thread
        threading.Thread(target=self.send_email, args=(error_message,), daemon=True).start()
        # Emit error_signal with the error message
        self.communicator.error_signal.emit(f"Error: {error_message}")

    def validate_email(self, email):
        """Validate email using regex"""
        regex = r'^[\w\.-]+@[\w\.-]+\.\w+$'
        return re.match(regex, email) is not None

    def send_email(self, error_message):
        try:
            smtp_server = self.email_config['smtp_server']
            smtp_port = self.email_config['smtp_port']
            sender_email = self.email_config['sender_email']
            sender_password = self.email_config['sender_password']
            receiver_email = self.email_config['receiver_email']

            subject = "ROS2 Error Notification"
            body = f"An error has occurred in the ROS2 system:\n\n{error_message}"

            # MIME setup
            msg = MIMEMultipart()
            msg['From'] = sender_email
            msg['To'] = receiver_email
            msg['Subject'] = subject
            msg.attach(MIMEText(body, 'plain'))

            # SMTP connection and send email
            server = smtplib.SMTP(smtp_server, smtp_port)
            server.starttls()
            server.login(sender_email, sender_password)
            text = msg.as_string()
            server.sendmail(sender_email, receiver_email, text)
            server.quit()

            self.get_logger().info(f'Error email sent to {receiver_email}.')
        except Exception as e:
            self.get_logger().error(f'Failed to send email: {e}')

class StateSubscriberNode(Node):
    def __init__(self, communicator: Communicator):
        super().__init__('state_subscriber')
        self.communicator = communicator
        self.subscription = self.create_subscription(
            Int32,
            'state',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('StateSubscriberNode has been initialized and subscribed to "state" topic.')

    def listener_callback(self, msg):
        state = msg.data
        self.get_logger().info(f"Received state: {state}")
        self.communicator.state_signal.emit(state)

# --- LoginWindow class ---
class LoginWindow(QWidget):
    def __init__(self, manipulation_node: ManipulationPublisherNode, communicator: Communicator, email_config: dict):
        super().__init__()
        self.manipulation_node = manipulation_node
        self.communicator = communicator
        self.email_config = email_config  # Store reference to email_config
        self.main_window = None
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('Login')
        self.setGeometry(100, 100, 300, 150)

        self.label_username = QLabel('Username:')
        self.input_username = QLineEdit()

        self.label_password = QLabel('Password:')
        self.input_password = QLineEdit()
        self.input_password.setEchoMode(QLineEdit.Password)

        self.button_login = QPushButton('Login')
        self.button_login.clicked.connect(self.handle_login)

        layout = QVBoxLayout()
        layout.addWidget(self.label_username)
        layout.addWidget(self.input_username)
        layout.addWidget(self.label_password)
        layout.addWidget(self.input_password)
        layout.addWidget(self.button_login)
        self.setLayout(layout)

    @pyqtSlot()
    def handle_login(self):
        username = self.input_username.text()
        password = self.input_password.text()

        # Simple username and password check
        if username == 'rokey' and password == 'rokey':
            QMessageBox.information(self, 'Login', 'Login Successful!')
            self.manipulation_node.get_logger().info(f'User {username} logged in successfully.')
            self.open_main_window()
        else:
            QMessageBox.warning(self, 'Login', 'Invalid username or password.')

    def open_main_window(self):
        self.main_window = MainWindow(self.manipulation_node, self.communicator, self.email_config)
        self.main_window.show()
        self.close()

# --- MainWindow class ---
class MainWindow(QMainWindow):
    image_received_signal = pyqtSignal(QImage)  # Signal to update the image in the GUI

    def __init__(self, manipulation_node: ManipulationPublisherNode, communicator: Communicator, email_config: dict):
        super().__init__()
        self.manipulation_node = manipulation_node
        self.communicator = communicator
        self.email_config = email_config  # Reference to email_config
        self.bridge = CvBridge()  # CV Bridge for image conversion
        self.init_ui()

        # Initialize operation time variables
        self.operation_timer = QTimer()
        self.operation_timer.timeout.connect(self.update_operation_time)  # 연결
        self.start_time = None
        self.elapsed_time = 0  # in seconds

        # Publishers
        qos_order = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_ALL,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.order_publisher = self.manipulation_node.create_publisher(String, '/order', qos_profile=qos_order)

        qos_belt = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )
        self.belt_publisher = self.manipulation_node.create_publisher(Bool, '/belt', qos_profile=qos_belt)

        # Subscriber for /image_topic
        self.image_subscription = self.manipulation_node.create_subscription(
            CompressedImage,
            '/image_topic',
            self.image_callback,
            10)
        self.image_subscription  # Prevent unused variable warning

        # Connect the signal to the slot
        self.image_received_signal.connect(self.update_image_label)

        # Connect communicator signals
        self.communicator.state_signal.connect(self.update_loading_bar)
        self.communicator.error_signal.connect(self.show_error_popup)

    def init_ui(self):
        self.setWindowTitle("터틀봇 조종 GUI")
        self.setGeometry(100, 100, 1200, 800)  # Initial window size

        # Main widget and layout
        self.main_widget = QWidget(self)
        self.setCentralWidget(self.main_widget)
        self.main_layout = QVBoxLayout()
        self.main_widget.setLayout(self.main_layout)

        # Bottom layout (video and other controls)
        self.bottom_layout = QHBoxLayout()
        self.main_layout.addLayout(self.bottom_layout)

        # Left side: Video display and loading bar
        self.left_layout = QVBoxLayout()
        self.bottom_layout.addLayout(self.left_layout)

        # Real-time webcam video display
        self.video_label = QLabel(self)
        self.video_label.setFixedSize(640, 480)  # Fixed size
        self.video_label.setStyleSheet("border: 1px solid black;")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.left_layout.addWidget(self.video_label)

        # Loading bar layout (below video)
        self.loading_layout = QVBoxLayout()
        self.left_layout.addLayout(self.loading_layout)

        # State loading bar
        self.loading_bar = QProgressBar(self)
        self.loading_bar.setMaximum(10)
        self.loading_bar.setValue(0)
        self.loading_bar.setTextVisible(False)
        self.loading_bar.setFixedHeight(30)
        self.loading_layout.addWidget(self.loading_bar)

        # Current state label
        self.state_label = QLabel("현재 상태: 명령 받았음 (0)", self)
        self.state_label.setAlignment(Qt.AlignCenter)
        self.state_label.setStyleSheet("font-size: 14px;")
        self.loading_layout.addWidget(self.state_label)

        # Add operation time label below state_label
        self.operation_time_label = QLabel("작동 시간: 0초", self)
        self.operation_time_label.setAlignment(Qt.AlignCenter)
        self.operation_time_label.setStyleSheet("font-size: 14px;")
        self.loading_layout.addWidget(self.operation_time_label)  # 새 QLabel 추가

        # Right side: Job list and controls
        self.right_layout = QVBoxLayout()
        self.bottom_layout.addLayout(self.right_layout)

        # Image display label (replacing Yolo Start Image placeholder)
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setText("No image received")
        self.image_label.setFixedSize(400, 300)  # Adjust dimensions as needed
        self.right_layout.addWidget(self.image_label)

        # Job list and time (now containing spin boxes)
        self.job_group = QGroupBox("작업 목록")
        self.job_layout = QVBoxLayout()
        self.job_group.setLayout(self.job_layout)
        self.right_layout.addWidget(self.job_group)

        # Clear existing job_layout contents
        # (If there were any existing widgets, remove them)
        for i in reversed(range(self.job_layout.count())):
            item = self.job_layout.itemAt(i)
            if item.widget():
                item.widget().deleteLater()
            elif item.layout():
                self.clear_layout(item.layout())

        # Red box layout
        red_layout = QVBoxLayout()
        red_label = QLabel('red box')
        red_label.setAlignment(Qt.AlignCenter)
        self.red_spinbox = QSpinBox()
        self.red_spinbox.setRange(0, 5)
        self.red_spinbox.setValue(0)
        red_layout.addWidget(red_label)
        red_layout.addWidget(self.red_spinbox)
        self.job_layout.addLayout(red_layout)

        # Blue box layout
        blue_layout = QVBoxLayout()
        blue_label = QLabel('blue box')
        blue_label.setAlignment(Qt.AlignCenter)
        self.blue_spinbox = QSpinBox()
        self.blue_spinbox.setRange(0, 5)
        self.blue_spinbox.setValue(0)
        blue_layout.addWidget(blue_label)
        blue_layout.addWidget(self.blue_spinbox)
        self.job_layout.addLayout(blue_layout)

        # Goto goal layout
        goto_layout = QVBoxLayout()
        goto_label = QLabel('goto goal')
        goto_label.setAlignment(Qt.AlignCenter)
        self.goto_spinbox = QSpinBox()
        self.goto_spinbox.setRange(1, 3)
        self.goto_spinbox.setValue(1)
        goto_layout.addWidget(goto_label)
        goto_layout.addWidget(self.goto_spinbox)
        self.job_layout.addLayout(goto_layout)

        # Send button
        self.send_button = QPushButton('send')
        self.send_button.clicked.connect(self.send_order)
        self.send_button.setFixedSize(100, 40)
        self.job_layout.addWidget(self.send_button, alignment=Qt.AlignCenter)

        # Control layout (manual control buttons and conveyor control)
        self.control_group = QGroupBox("수동 조종")
        self.control_layout = QVBoxLayout()
        self.control_group.setLayout(self.control_layout)
        self.right_layout.addWidget(self.control_group)

        # Manual control buttons
        self.manual_controls_layout = QHBoxLayout()
        self.control_layout.addLayout(self.manual_controls_layout)

        self.play_button = QPushButton("Play", self)
        self.play_button.setFixedSize(100, 50)
        self.manual_controls_layout.addWidget(self.play_button)

        self.stop_button = QPushButton("Stop", self)
        self.stop_button.setFixedSize(100, 50)
        self.manual_controls_layout.addWidget(self.stop_button)

        self.pause_button = QPushButton("Pause", self)
        self.pause_button.setFixedSize(100, 50)
        self.manual_controls_layout.addWidget(self.pause_button)

        self.resume_button = QPushButton("Resume", self)
        self.resume_button.setFixedSize(100, 50)
        self.manual_controls_layout.addWidget(self.resume_button)

        self.reset_button = QPushButton("Reset", self)
        self.reset_button.setFixedSize(100, 50)
        self.manual_controls_layout.addWidget(self.reset_button)

        # Conveyor control buttons (replacing with On/Off buttons from original user_gui)
        self.conveyor_group = QGroupBox("컨베이어 조작")
        self.conveyor_layout = QHBoxLayout()
        self.conveyor_group.setLayout(self.conveyor_layout)
        self.control_layout.addWidget(self.conveyor_group)

        # On button
        self.on_button = QPushButton('On')
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
        self.conveyor_layout.addWidget(self.on_button)

        # Off button
        self.off_button = QPushButton('Off')
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
        self.conveyor_layout.addWidget(self.off_button)

        # --- Email Input Section ---
        self.email_group = QGroupBox("Email 설정")
        self.email_layout = QHBoxLayout()
        self.email_group.setLayout(self.email_layout)

        self.email_label = QLabel("Email:")
        self.email_input = QLineEdit()
        self.email_input.setPlaceholderText("Enter receiver email")
        self.email_input.textChanged.connect(self.on_email_changed)  # Connect to re-enable check button

        self.check_button = QPushButton("Check")
        self.check_button.clicked.connect(self.check_email)

        self.email_layout.addWidget(self.email_label)
        self.email_layout.addWidget(self.email_input)
        self.email_layout.addWidget(self.check_button)

        self.control_layout.addWidget(self.email_group)  # Add email group below conveyor control

        # Timer setup (update job time)
        self.start_time = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_time)

        # Connect buttons to functions
        self.play_button.clicked.connect(self.play_job)
        self.stop_button.clicked.connect(self.stop_job)
        self.pause_button.clicked.connect(self.pause_job)
        self.resume_button.clicked.connect(self.resume_job)
        self.reset_button.clicked.connect(self.reset_job)

        # Start video thread
        self.video_thread = VideoThread()
        self.video_thread.change_pixmap_signal.connect(self.update_video_image)
        self.video_thread.start()

    def clear_layout(self, layout):
        if layout is not None:
            while layout.count():
                child = layout.takeAt(0)
                if child.widget():
                    child.widget().deleteLater()
                elif child.layout():
                    self.clear_layout(child.layout())

    def image_callback(self, msg):
        try:
            # Convert CompressedImage message to OpenCV image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                self.manipulation_node.get_logger().error("Received empty image")
                return

            # Convert OpenCV image (BGR) to RGB format
            cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            height, width, channel = cv_image_rgb.shape
            bytes_per_line = 3 * width

            # Convert to QImage
            q_image = QImage(cv_image_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)

            # Emit the signal to update the image in the GUI
            self.image_received_signal.emit(q_image)
        except Exception as e:
            self.manipulation_node.get_logger().error(f"Could not convert image: {e}")

    def update_image_label(self, q_image):
        # Update the image_label with the new image
        pixmap = QPixmap.fromImage(q_image)
        pixmap = pixmap.scaled(self.image_label.width(), self.image_label.height(), Qt.KeepAspectRatio)
        self.image_label.setPixmap(pixmap)

    def update_video_image(self, qt_img):
        # Update the video_label with the new image
        self.video_label.setPixmap(QPixmap.fromImage(qt_img).scaled(
            self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def send_order(self):
        red_value = self.red_spinbox.value()
        blue_value = self.blue_spinbox.value()
        goto_value = self.goto_spinbox.value()
        order_str = f'red_box:{red_value},blue_box:{blue_value},goto_goal:{goto_value}'
        msg = String()
        msg.data = order_str
        self.order_publisher.publish(msg)
        self.manipulation_node.get_logger().info(f'Published to /order: {order_str}')

    def belt_on(self):
        msg = Bool()
        msg.data = True
        self.belt_publisher.publish(msg)
        self.manipulation_node.get_logger().info('Published to /belt: True')

    def belt_off(self):
        msg = Bool()
        msg.data = False
        self.belt_publisher.publish(msg)
        self.manipulation_node.get_logger().info('Published to /belt: False')

    def update_time(self):
        if self.start_time:
            elapsed = int(time.time() - self.start_time)
            # Update operation time label
            self.operation_time_label.setText(f"작동 시간: {elapsed}초")

    def play_job(self):
        QMessageBox.information(self, "Play", "작업을 시작합니다.")
        self.publish_manipulation_command("play")  # Publish "play" command

    def stop_job(self):
        QMessageBox.information(self, "Stop", "작업을 중지합니다.")
        self.publish_manipulation_command("stop")  # Publish "stop" command

    def pause_job(self):
        QMessageBox.information(self, "Pause", "작업을 일시 중지합니다.")
        self.publish_manipulation_command("pause")  # Publish "pause" command

    def resume_job(self):
        QMessageBox.information(self, "Resume", "작업을 재개합니다.")
        self.publish_manipulation_command("resume")  # Publish "resume" command

    def reset_job(self):
        QMessageBox.information(self, "Reset", "작업을 초기화합니다.")
        # Reset any job-related state
        self.publish_manipulation_command("reset")  # Publish "reset" command

    def publish_manipulation_command(self, command: str):
        self.manipulation_node.publish_command(command)

    def closeEvent(self, event):
        """Close the window and stop threads"""
        self.video_thread.stop()
        event.accept()

    def update_loading_bar(self, state):
        """Update the loading bar and state label based on the state"""
        state_descriptions = {
            0: "명령 받았음",
            1: "이동 시작",
            2: "피킹 장소 도착",
            3: "박스 잡기 완료",
            4: "컨베이어 벨트 올리기 중",
            5: "이동 중",
            6: "바구니 장소 도착",
            7: "바구니 잡는 중",
            8: "이동 중",
            9: "하역장 도착 완료",
            10: "하차 완료"
        }

        if state in state_descriptions:
            description = state_descriptions[state]
            self.loading_bar.setValue(state)
            self.state_label.setText(f"현재 상태: {description} ({state})")

            if state == 0:
                # Reset operation time
                self.elapsed_time = 0
                self.operation_time_label.setText(f"작동 시간: {self.elapsed_time}초")
                self.operation_timer.stop()

            if state == 1 and self.start_time is None:
                # Start operation timer on first state change (state 1)
                self.start_time = time.time()
                self.elapsed_time = 0
                self.operation_time_label.setText(f"작동 시간: {self.elapsed_time}초")
                self.operation_timer.start(1000)  # Update every second

            if state == 10:
                # Stop operation timer on state 10
                self.operation_timer.stop()
                self.manipulation_node.get_logger().info(f"작동 시간: {self.elapsed_time}초")
        else:
            self.loading_bar.setValue(0)
            self.state_label.setText("현재 상태: 알 수 없음")

    def show_error_popup(self, message):
        QMessageBox.critical(self, 'Error', message)

    # --- Email Check Functionality ---
    def check_email(self):
        email = self.email_input.text().strip()
        if self.validate_email(email):
            # Email is valid
            self.email_config['receiver_email'] = email
            QMessageBox.information(self, 'Email Check', 'Email is valid and set successfully!')
            self.check_button.setEnabled(False)  # Disable the check button
        else:
            # Email is invalid
            QMessageBox.warning(self, 'Invalid Email', 'Please enter a valid email address.')

    def validate_email(self, email):
        """Validate email using regex"""
        regex = r'^[\w\.-]+@[\w\.-]+\.\w+$'
        return re.match(regex, email) is not None

    def on_email_changed(self, text):
        """Re-enable the check button if email input is modified"""
        self.check_button.setEnabled(True)

    def update_operation_time(self):
        self.elapsed_time += 1
        self.operation_time_label.setText(f"작동 시간: {self.elapsed_time}초")
def main():
    load_dotenv()  # Load .env file if present

    # Initialize ROS2
    rclpy.init(args=None)

    # Email configuration from environment variables
    email_config = {
        'smtp_server': os.getenv('SMTP_SERVER', 'smtp.gmail.com'),         # SMTP server address
        'smtp_port': int(os.getenv('SMTP_PORT', 587)),                     # SMTP port
        'sender_email': os.getenv('SENDER_EMAIL', 'kwilee0426@gmail.com'), # Sender email address
        'sender_password': os.getenv('SENDER_PASSWORD', 'dgnbwiqaizoekfvd'),               # Sender email password or app password
        'receiver_email': '',                                              # Initially empty, to be set via GUI
    }

    # Create Communicator object
    communicator = Communicator()

    # Create ROS2 nodes
    manipulation_node = ManipulationPublisherNode()
    manipulation_listener_node = ManipulationListenerNode()
    error_subscriber_node = ErrorSubscriberNode(email_config, communicator)
    state_subscriber_node = StateSubscriberNode(communicator)

    # Create a generic ROS2 node for MainWindow
    ros_node = Node('simple_order_gui_node')

    # Create an executor and add all nodes
    executor = MultiThreadedExecutor()
    executor.add_node(manipulation_node)
    executor.add_node(manipulation_listener_node)
    executor.add_node(error_subscriber_node)
    executor.add_node(state_subscriber_node)
    executor.add_node(ros_node)  # Add the node used in MainWindow

    # Create and start ROS spinning in a separate thread
    def spin_executor():
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass

    spin_thread = threading.Thread(target=spin_executor, daemon=True)
    spin_thread.start()

    # Create PyQt5 application
    app = QApplication(sys.argv)

    # Create LoginWindow
    login_window = LoginWindow(manipulation_node, communicator, email_config)
    login_window.show()

    # Execute the application
    exit_code = app.exec_()

    # Shutdown ROS
    executor.shutdown()
    manipulation_node.destroy_node()
    manipulation_listener_node.destroy_node()
    error_subscriber_node.destroy_node()
    state_subscriber_node.destroy_node()
    ros_node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)

if __name__ == '__main__':
    main()
