import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QLineEdit, QPushButton, QListWidget,
    QListWidgetItem, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QMessageBox,
    QSizePolicy, QFrame
)
from PyQt5.QtCore import pyqtSlot, QTimer, Qt, pyqtSignal, QObject, QThread
from PyQt5.QtGui import QImage, QPixmap, QFont
import cv2
import time
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
import threading
import os
from dotenv import load_dotenv

class Communicator(QObject):
    error_signal = pyqtSignal(str)  # 오류 메시지를 전달할 시그널

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(QImage)

    def __init__(self, source=0):
        super().__init__()
        self.source = source
        self._run_flag = True

    def run(self):
        # 비디오 소스 열기
        cap = cv2.VideoCapture(self.source)
        while self._run_flag:
            ret, cv_img = cap.read()
            if ret:
                # 이미지를 RGB 형식으로 변환
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                # QImage로 변환
                height, width, channel = cv_img.shape
                bytes_per_line = 3 * width
                qt_img = QImage(cv_img.data, width, height, bytes_per_line, QImage.Format_RGB888)
                # 신호 발생
                self.change_pixmap_signal.emit(qt_img)
        # 비디오 소스 해제
        cap.release()

    def stop(self):
        self._run_flag = False
        self.wait()

class LoginWindow(QWidget):
    def __init__(self, node: Node, app, communicator: Communicator):
        super().__init__()
        self.node = node
        self.app = app  # QApplication 인스턴스를 참조
        self.communicator = communicator
        self.communicator.error_signal.connect(self.show_error_popup)
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
        if username == 'admin' and password == 'password':
            QMessageBox.information(self, 'Login', 'Login Successful!')
            self.node.get_logger().info(f'User {username} logged in successfully.')
            self.open_main_window()
        else:
            QMessageBox.warning(self, 'Login', 'Invalid username or password.')

    def open_main_window(self):
        self.main_window = MainWindow(self.node, self.communicator)  # MainWindow 인스턴스 생성
        self.main_window.show()
        self.close()  # 로그인 창 닫기

    @pyqtSlot(str)
    def show_error_popup(self, message):
        QMessageBox.critical(self, 'Error', message)

class MainWindow(QMainWindow):
    def __init__(self, node: Node, communicator: Communicator):
        super().__init__()
        self.node = node
        self.communicator = communicator
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("터틀봇 조종 GUI")
        self.setGeometry(100, 100, 1200, 800)  # 초기 윈도우 크기

        # 메인 위젯 및 메인 레이아웃
        self.main_widget = QWidget(self)
        self.setCentralWidget(self.main_widget)
        self.main_layout = QVBoxLayout()
        self.main_widget.setLayout(self.main_layout)

        # 상단 레이아웃 (실시간 영상과 YOLO 이미지)
        self.top_layout = QHBoxLayout()
        self.main_layout.addLayout(self.top_layout)

        # 실시간 웹캠 영상 표시
        self.video_label = QLabel(self)
        self.video_label.setFixedSize(640, 480)  # 고정 크기 설정
        self.video_label.setStyleSheet("border: 1px solid black;")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.top_layout.addWidget(self.video_label)

        # YOLO 이미지 레이아웃
        self.yolo_layout = QVBoxLayout()
        self.top_layout.addLayout(self.yolo_layout)

        # YOLO 시작 이미지
        self.yolo_start_label = QLabel("YOLO Start Image", self)
        self.yolo_start_label.setFixedSize(320, 240)
        self.yolo_start_label.setStyleSheet("border: 1px solid black;")
        self.yolo_start_label.setAlignment(Qt.AlignCenter)
        self.yolo_layout.addWidget(self.yolo_start_label)

        # YOLO 종료 이미지
        self.yolo_end_label = QLabel("YOLO End Image", self)
        self.yolo_end_label.setFixedSize(320, 240)
        self.yolo_end_label.setStyleSheet("border: 1px solid black;")
        self.yolo_end_label.setAlignment(Qt.AlignCenter)
        self.yolo_layout.addWidget(self.yolo_end_label)

        # 중앙 레이아웃 (작업 목록 및 시간)
        self.middle_layout = QHBoxLayout()
        self.main_layout.addLayout(self.middle_layout)

        # 작업 목록과 소요 시간
        self.job_group = QGroupBox("작업 목록")
        self.job_layout = QVBoxLayout()
        self.job_group.setLayout(self.job_layout)
        self.middle_layout.addWidget(self.job_group, 2)

        self.job_list = QListWidget(self)
        self.job_list.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # 폰트 크기 설정
        font = QFont()
        font.setPointSize(14)  # 원하는 폰트 크기로 설정 (예: 14)
        self.job_list.setFont(font)
        
        # 또는 스타일 시트를 사용하여 폰트 크기 설정
        # self.job_list.setStyleSheet("font-size: 14pt;")
        
        self.job_layout.addWidget(self.job_list)

        self.job_time_label = QLabel("소요 시간: 0초", self)
        self.job_time_label.setAlignment(Qt.AlignCenter)
        self.job_time_label.setStyleSheet("font-size: 16px;")
        self.job_layout.addWidget(self.job_time_label)

        self.populate_jobs()

        # 컨트롤 레이아웃 (수동 조종 버튼 및 컨베이어 조종)
        self.control_group = QGroupBox("수동 조종")
        self.control_layout = QVBoxLayout()
        self.control_group.setLayout(self.control_layout)
        self.middle_layout.addWidget(self.control_group, 1)

        # 수동 조종 버튼
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

        # 컨베이어 수동 조종 버튼
        self.conveyor_group = QGroupBox("컨베이어 조작")
        self.conveyor_layout = QHBoxLayout()
        self.conveyor_group.setLayout(self.conveyor_layout)
        self.control_layout.addWidget(self.conveyor_group)

        self.conveyor_on_button = QPushButton("On", self)
        self.conveyor_on_button.setFixedSize(100, 50)
        self.conveyor_layout.addWidget(self.conveyor_on_button)

        self.conveyor_off_button = QPushButton("Off", self)
        self.conveyor_off_button.setFixedSize(100, 50)
        self.conveyor_layout.addWidget(self.conveyor_off_button)

        # 타이머 설정 (작업 소요 시간 업데이트)
        self.start_time = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_time)

        # 버튼 이벤트 연결
        self.play_button.clicked.connect(self.play_job)
        self.stop_button.clicked.connect(self.stop_job)
        self.pause_button.clicked.connect(self.pause_job)
        self.resume_button.clicked.connect(self.resume_job)
        self.reset_button.clicked.connect(self.reset_job)
        self.conveyor_on_button.clicked.connect(self.conveyor_on)
        self.conveyor_off_button.clicked.connect(self.conveyor_off)
        self.job_list.currentItemChanged.connect(self.select_job)

        # 비디오 스레드 시작
        self.thread = VideoThread()
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.start()

    def populate_jobs(self):
        jobs = [
            ("Job1", "red*2, blue*1, goto goal 1"),
            ("Job2", "red*1, blue*2, goto goal 2"),
            ("Job3", "red*1, goto goal 3"),
        ]
        for name, details in jobs:
            item = QListWidgetItem(f"{name}: {details}")
            self.job_list.addItem(item)

    def select_job(self, current, previous):
        if current:
            QMessageBox.information(self, "작업 선택", f"선택된 작업: {current.text()}")
            # 작업 시작 시점에 타이머 시작
            self.start_time = time.time()
            self.timer.start(1000)  # 1초마다 업데이트

    def update_time(self):
        if self.start_time:
            elapsed = int(time.time() - self.start_time)
            self.job_time_label.setText(f"소요 시간: {elapsed}초")

    def play_job(self):
        QMessageBox.information(self, "Play", "작업을 시작합니다.")
        # ROS 2 작업 시작 코드 삽입

    def stop_job(self):
        QMessageBox.information(self, "Stop", "작업을 중지합니다.")
        # ROS 2 작업 중지 코드 삽입

    def pause_job(self):
        QMessageBox.information(self, "Pause", "작업을 일시 중지합니다.")
        # ROS 2 작업 일시 중지 코드 삽입

    def resume_job(self):
        QMessageBox.information(self, "Resume", "작업을 재개합니다.")
        # ROS 2 작업 재개 코드 삽입

    def reset_job(self):
        QMessageBox.information(self, "Reset", "작업을 초기화합니다.")
        self.job_time_label.setText("소요 시간: 0초")
        self.timer.stop()
        self.start_time = None
        # ROS 2 작업 초기화 코드 삽입

    def conveyor_on(self):
        QMessageBox.information(self, "Conveyor", "컨베이어를 켭니다.")
        # 컨베이어 On 코드 삽입

    def conveyor_off(self):
        QMessageBox.information(self, "Conveyor", "컨베이어를 끕니다.")
        # 컨베이어 Off 코드 삽입

    def closeEvent(self, event):
        """창이 닫힐 때 스레드를 종료"""
        self.thread.stop()
        event.accept()

    def update_image(self, qt_img):
        self.video_label.setPixmap(QPixmap.fromImage(qt_img).scaled(
            self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

class ErrorSubscriber(Node):
    def __init__(self, email_config, communicator: Communicator):
        super().__init__('error_subscriber')
        self.subscription = self.create_subscription(
            String,  # 메시지 타입
            '/error',  # 토픽 이름
            self.listener_callback,
            10  # QoS 프로파일 (큐 사이즈)
        )
        self.subscription  # prevent unused variable warning
        self.email_config = email_config
        self.communicator = communicator
        self.get_logger().info('ErrorSubscriber initialized and subscribed to /error topic.')

    def listener_callback(self, msg):
        error_message = msg.data
        self.get_logger().error(f'Received error message: {error_message}')
        # 이메일 발송을 별도의 스레드에서 처리하여 블로킹 방지
        threading.Thread(target=self.send_email, args=(error_message,)).start()
        # 팝업 창 표시를 위한 시그널 발송
        self.communicator.error_signal.emit(error_message)

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

def main(args=None):
    load_dotenv()  # .env 파일 로드
    rclpy.init(args=args)

    # 이메일 설정
    email_config = {
        'smtp_server': os.getenv('SMTP_SERVER', 'smtp.gmail.com'),       # SMTP 서버 주소
        'smtp_port': int(os.getenv('SMTP_PORT', 587)),                  # SMTP 포트
        'sender_email': os.getenv('SENDER_EMAIL'),                      # 발신자 이메일 주소
        'sender_password': os.getenv('SENDER_PASSWORD'),                # 발신자 이메일 비밀번호 또는 앱 비밀번호
        'receiver_email': os.getenv('RECEIVER_EMAIL'),                  # 수신자 이메일 주소
    }

    # 커뮤니케이터 객체 생성
    communicator = Communicator()

    # ROS2 노드 생성
    login_node = Node('login_node')

    # ErrorSubscriber 노드 생성
    error_subscriber = ErrorSubscriber(email_config, communicator)

    # PyQt5 애플리케이션 생성
    app = QApplication(sys.argv)
    login_window = LoginWindow(login_node, app, communicator)
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
