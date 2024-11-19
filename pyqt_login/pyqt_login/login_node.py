import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QPushButton, QVBoxLayout, QMessageBox
)
from PyQt5.QtCore import pyqtSlot, QTimer, Qt
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
import threading

class LoginWindow(QWidget):
    def __init__(self, node: Node, app):
        super().__init__()
        self.node = node
        self.app = app  # QApplication 인스턴스를 참조
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

        # 간단한 예제로 사용자 이름과 비밀번호를 확인합니다.
        if username == 'rokey' and password == 'rokey':
            QMessageBox.information(self, 'Login', 'Login Successful!')
            self.node.get_logger().info(f'User {username} logged in successfully.')
            self.open_main_window()
        else:
            QMessageBox.warning(self, 'Login', 'Invalid username or password.')

    def open_main_window(self):
        self.main_window = MainWindow(self.node)  # MainWindow 인스턴스 생성
        self.main_window.show()
        self.close()  # 로그인 창 닫기

class MainWindow(QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('Main Window')
        self.setGeometry(100, 100, 400, 200)

        self.label_welcome = QLabel('Welcome to the Main Window!')
        self.label_welcome.setAlignment(Qt.AlignCenter)
        self.button_exit = QPushButton('Exit')
        self.button_exit.clicked.connect(self.handle_exit)

        layout = QVBoxLayout()
        layout.addWidget(self.label_welcome)
        layout.addStretch()
        layout.addWidget(self.button_exit)

        self.setLayout(layout)

    @pyqtSlot()
    def handle_exit(self):
        self.node.get_logger().info('Exiting application.')
        self.close()
        rclpy.shutdown()
        QApplication.quit()

class ErrorSubscriber(Node):
    def __init__(self, email_config):
        super().__init__('error_subscriber')
        self.subscription = self.create_subscription(
            String,  # 메시지 타입
            '/error',  # 토픽 이름
            self.listener_callback,
            10  # QoS 프로파일 (큐 사이즈)
        )
        self.subscription  # prevent unused variable warning
        self.email_config = email_config
        self.get_logger().info('ErrorSubscriber initialized and subscribed to /error topic.')

    def listener_callback(self, msg):
        error_message = msg.data
        self.get_logger().error(f'Received error message: {error_message}')
        # 이메일 발송을 별도의 스레드에서 처리하여 블로킹 방지
        threading.Thread(target=self.send_email, args=(error_message,)).start()

    def send_email(self, error_message):
        try:
            smtp_server = self.email_config['smtp_server']
            smtp_port = self.email_config['smtp_port']
            sender_email = self.email_config['sender_email']
            sender_password = self.email_config['sender_password']
            receiver_email = self.email_config['receiver_email']

            subject = "ROS2 Error Notification"
            body = f"An error has occurred in the ROS2 system:\n\n{error_message}"

            # MIME 설정
            msg = MIMEMultipart()
            msg['From'] = sender_email
            msg['To'] = receiver_email
            msg['Subject'] = subject
            msg.attach(MIMEText(body, 'plain'))

            # SMTP 서버에 연결하여 이메일 발송
            server = smtplib.SMTP(smtp_server, smtp_port)
            server.starttls()
            server.login(sender_email, sender_password)
            text = msg.as_string()
            server.sendmail(sender_email, receiver_email, text)
            server.quit()

            self.get_logger().info(f'Error email sent to {receiver_email}.')
        except Exception as e:
            self.get_logger().error(f'Failed to send email: {e}')

from std_msgs.msg import String  # ROS2 String 메시지 타입

def main(args=None):
    rclpy.init(args=args)

    # 이메일 설정
    email_config = {
        'smtp_server': 'smtp.gmail.com',       # SMTP 서버 주소 (예: 'smtp.gmail.com')
        'smtp_port': 587,                        # SMTP 포트 (일반적으로 587)
        'sender_email': 'kwilee0426@gmail.com',  # 발신자 이메일 주소
        'sender_password': 'dgnbwiqaizoekfvd',        # 발신자 이메일 비밀번호 또는 앱 비밀번호
        'receiver_email': 'ssm06081@gmail.com',  # 수신자 이메일 주소
    }

    # ROS2 노드 생성
    login_node = Node('login_node')

    # ErrorSubscriber 노드 생성
    error_subscriber = ErrorSubscriber(email_config)

    # PyQt5 애플리케이션 생성
    app = QApplication(sys.argv)
    login_window = LoginWindow(login_node, app)
    login_window.show()

    # ROS2 스피닝을 PyQt5 이벤트 루프와 통합
    # QTimer를 사용하여 주기적으로 ROS2 spin_once를 호출합니다.
    def ros_spin():
        rclpy.spin_once(login_node, timeout_sec=0)
        rclpy.spin_once(error_subscriber, timeout_sec=0)

    timer = QTimer()
    timer.timeout.connect(ros_spin)
    timer.start(100)  # 100ms마다 spin_once 호출

    # PyQt5 이벤트 루프 실행
    exit_code = app.exec_()

    # 애플리케이션 종료 시 ROS2 정리
    login_node.destroy_node()
    error_subscriber.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
