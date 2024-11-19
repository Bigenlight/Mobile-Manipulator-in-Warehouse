import sys
import cv2
import time
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QListWidget, QListWidgetItem,
    QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QMessageBox, QSizePolicy, QFrame
)

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

class MyWindow(QMainWindow):
    def __init__(self):
        super().__init__()
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

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())
