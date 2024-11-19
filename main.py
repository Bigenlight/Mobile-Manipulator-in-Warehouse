import sys
import cv2
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # 메시지 타입에 맞게 변경하세요
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QListWidget, QListWidgetItem,
    QVBoxLayout, QHBoxLayout, QGroupBox, QMessageBox, QSizePolicy, QProgressBar
)
from PyQt5 import uic  # .ui 파일 로드를 위한 임포트

# UI 파일 경로 설정
UI_FILE = "/home/rokey/3_ws/src/main_window.ui"

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

class ROSSubscriberThread(QThread):
    state_signal = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self.node = None

    def run(self):
        rclpy.init(args=None)
        self.node = StateSubscriberNode(self.listener_callback)
        try:
            rclpy.spin(self.node)
        except Exception as e:
            print(f"ROS Subscriber encountered an exception: {e}")
        finally:
            self.node.destroy_node()
            rclpy.shutdown()

    def listener_callback(self, msg):
        print(f"Received state: {msg.data}")  # 디버깅용 출력
        self.state_signal.emit(msg.data)

    def stop(self):
        if self.node:
            self.node.destroy_node()
        rclpy.shutdown()
        self.quit()
        self.wait()

class StateSubscriberNode(Node):
    def __init__(self, callback):
        super().__init__('state_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'state',
            callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('StateSubscriberNode has been initialized and subscribed to "state" topic.')

class MyWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        # UI 파일 로드
        uic.loadUi(UI_FILE, self)

        # 위젯 참조
        self.videoLabel = self.findChild(QLabel, "videoLabel")
        self.yoloStartLabel = self.findChild(QLabel, "yoloStartLabel")
        self.yoloEndLabel = self.findChild(QLabel, "yoloEndLabel")
        self.loadingBar = self.findChild(QProgressBar, "loadingBar")
        self.stateLabel = self.findChild(QLabel, "stateLabel")
        self.jobList = self.findChild(QListWidget, "jobList")
        self.jobTimeLabel = self.findChild(QLabel, "jobTimeLabel")
        self.playButton = self.findChild(QPushButton, "playButton")
        self.stopButton = self.findChild(QPushButton, "stopButton")
        self.pauseButton = self.findChild(QPushButton, "pauseButton")
        self.resumeButton = self.findChild(QPushButton, "resumeButton")
        self.resetButton = self.findChild(QPushButton, "resetButton")
        self.conveyorOnButton = self.findChild(QPushButton, "conveyorOnButton")
        self.conveyorOffButton = self.findChild(QPushButton, "conveyorOffButton")

        # 로딩바 설정
        self.loadingBar.setMaximum(10)
        self.loadingBar.setValue(0)
        self.loadingBar.setTextVisible(False)

        # 작업 목록 초기화
        self.populate_jobs()

        # 타이머 설정 (작업 소요 시간 업데이트)
        self.start_time = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_time)

        # 버튼 이벤트 연결
        self.playButton.clicked.connect(self.play_job)
        self.stopButton.clicked.connect(self.stop_job)
        self.pauseButton.clicked.connect(self.pause_job)
        self.resumeButton.clicked.connect(self.resume_job)
        self.resetButton.clicked.connect(self.reset_job)
        self.conveyorOnButton.clicked.connect(self.conveyor_on)
        self.conveyorOffButton.clicked.connect(self.conveyor_off)
        self.jobList.currentItemChanged.connect(self.select_job)

        # 비디오 스레드 시작
        self.thread = VideoThread()
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.start()

        # ROS 2 구독자 스레드 시작
        self.ros_thread = ROSSubscriberThread()
        self.ros_thread.state_signal.connect(self.update_loading_bar)
        self.ros_thread.start()

    def populate_jobs(self):
        jobs = [
            ("Job1", "red*2, blue*1, goto goal 1"),
            ("Job2", "red*1, blue*2, goto goal 2"),
            ("Job3", "red*1, goto goal 3"),
        ]
        for name, details in jobs:
            item = QListWidgetItem(f"{name}: {details}")
            self.jobList.addItem(item)

    def select_job(self, current, previous):
        if current:
            QMessageBox.information(self, "작업 선택", f"선택된 작업: {current.text()}")
            # 작업 시작 시점에 타이머 시작
            self.start_time = time.time()
            self.timer.start(1000)  # 1초마다 업데이트

    def update_time(self):
        if self.start_time:
            elapsed = int(time.time() - self.start_time)
            self.jobTimeLabel.setText(f"소요 시간: {elapsed}초")

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
        self.jobTimeLabel.setText("소요 시간: 0초")
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
        self.ros_thread.stop()
        event.accept()

    def update_image(self, qt_img):
        self.videoLabel.setPixmap(QPixmap.fromImage(qt_img).scaled(
            self.videoLabel.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def update_loading_bar(self, state):
        """로딩바와 상태 레이블을 업데이트하는 메서드"""
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
            self.loadingBar.setValue(state)
            self.stateLabel.setText(f"현재 상태: {description} ({state})")
        else:
            self.loadingBar.setValue(0)
            self.stateLabel.setText("현재 상태: 알 수 없음")

if __name__ == "__main__":
    # Qt 플랫폼 플러그인 설정 (wayland 오류 해결)
    import os
    os.environ["QT_QPA_PLATFORM"] = "xcb"

    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())
