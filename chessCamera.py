import cv2
import numpy as np
import os

# 체커보드의 내부 코너 수 (가로 x 세로)
CHECKERBOARD = (6, 9)  # 체스보드의 내부 코너 개수
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# 각 체커보드 이미지에 대한 3D 점 벡터를 저장할 벡터 생성
objpoints = []  # 3D 점의 세계 좌표
imgpoints = []  # 2D 점의 이미지 좌표

# 3D 점의 세계 좌표 정의
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# 웹캠 초기화
cap = cv2.VideoCapture(2)  # 0번 웹캠 (필요 시 1, 2 등 다른 번호로 변경)

if not cap.isOpened():
    print("웹캠을 열 수 없습니다. 연결 상태를 확인하세요.")
    exit()

print("웹캠이 열렸습니다. 체스보드를 카메라에 보여주세요.")
print("10개의 체스보드 이미지가 수집되면 캘리브레이션을 수행합니다.")

collected_images = 0  # 수집된 체스보드 이미지 개수
required_images = 10  # 캘리브레이션에 필요한 체스보드 이미지 개수

while collected_images < required_images:
    ret, frame = cap.read()
    if not ret:
        print("웹캠에서 프레임을 읽을 수 없습니다. 다시 시도하세요.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 체스보드 코너 찾기
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
                                             cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ret:
        # 픽셀 좌표 미세조정
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # 코너를 감지한 경우 저장
        objpoints.append(objp)
        imgpoints.append(corners2)

        # 코너를 그려 화면에 표시
        cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret)
        collected_images += 1
        print(f"체스보드 이미지 수집 {collected_images}/{required_images} 완료")

    # 화면에 출력
    cv2.imshow('Calibration', frame)

    # ESC 키로 종료
    if cv2.waitKey(1) & 0xFF == 27:
        print("사용자가 종료했습니다.")
        break

# 웹캠 릴리즈 및 창 닫기
cap.release()
cv2.destroyAllWindows()

if collected_images < required_images:
    print("캘리브레이션에 충분한 데이터를 수집하지 못했습니다.")
    exit()

# 캘리브레이션 수행
print("캘리브레이션을 수행합니다...")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# 결과 출력
print("\n캘리브레이션 완료")
print("카메라 행렬 (mtx):\n", mtx)
print("왜곡 계수 (dist):\n", dist)

# 결과 저장
calibration_matrix_path = './calibration_matrix_webcam.npy'
distortion_coefficients_path = './distortion_coefficients_webcam.npy'

np.save(calibration_matrix_path, mtx)
np.save(distortion_coefficients_path, dist)

print(f"카메라 행렬이 {calibration_matrix_path}에 저장되었습니다.")
print(f"왜곡 계수가 {distortion_coefficients_path}에 저장되었습니다.")
