import cv2
import numpy as np

# ArUco 딕셔너리 선택 (5x5, 250개 마커)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)

# OpenCV 버전에 따라 DetectorParameters 생성 방식 변경
if hasattr(cv2.aruco, 'DetectorParameters_create'):
    aruco_params = cv2.aruco.DetectorParameters_create()
else:
    aruco_params = cv2.aruco.DetectorParameters()

# 카메라 캘리브레이션 파라미터 (실제 카메라에 맞게 수정 필요)
camera_matrix = np.array([
    [800, 0, 320],
    [0, 800, 240],
    [0, 0, 1]
], dtype=np.float64)

dist_coeffs = np.array([0.1, -0.05, 0, 0], dtype=np.float64)  # 예제 왜곡 계수

# ArUco 마커의 실제 크기 (미터 단위)
marker_length = 0.0105  # 1.05cm

# 웹캠 캡처 시작 (장치 인덱스 0으로 시도)
cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

print("카메라가 열렸습니다. ArUco 마커(ID42)를 카메라에 보여주세요.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("카메라 프레임을 읽을 수 없습니다.")
        break

    # 그레이스케일 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # ArUco 마커 탐지
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    # 마커 감지 여부 출력
    if ids is None:
        print("마커를 감지하지 못했습니다.")
    else:
        detected_ids = ids.flatten()
        print(f"감지된 마커 ID: {detected_ids}")

    # 감지된 마커 시각화
    if ids is not None:
        # 탐지된 모든 마커에 대해 Pose 추정
        for i, (corner, marker_id) in enumerate(zip(corners, ids.flatten())):
            if marker_id != 42:
                continue  # ID42 마커만 처리

            # Pose 추정
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corner, marker_length, camera_matrix, dist_coeffs)

            # Pose 추정 결과는 리스트 형태이므로, 첫 번째 마커만 사용
            rvec = rvecs[0][0]
            tvec = tvecs[0][0]

            # 상대 좌표 출력 (카메라 좌표계 기준)
            x, y, z = tvec
            distance = np.linalg.norm(tvec)
            print(f"Marker ID: {marker_id} | Position: x={x:.3f}, y={y:.3f}, z={z:.3f} | Distance: {distance:.3f}m")

            # Pose 시각화 (마커와 축 그리기)
            cv2.aruco.drawDetectedMarkers(frame, [corner], ids)
            cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_length)

            # 텍스트 오버레이 설정
            # 마커의 첫 번째 코너의 좌표를 사용하여 텍스트 위치 결정
            corner_position = tuple(corner[0][0].astype(int))
            text_x, text_y = corner_position

            # 텍스트 위치 조정: 마커 위에 텍스트가 위치하도록 y 좌표를 감소
            # 텍스트가 화면 밖으로 나가지 않도록 확인
            if text_y - 60 > 10:
                text_y -= 60
            else:
                text_y += 60

            # 상대 좌표 텍스트
            coord_text = f"ID:{marker_id} x:{x:.2f} y:{y:.2f} z:{z:.2f}"
            cv2.putText(frame, coord_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 거리 텍스트
            distance_text = f"Distance: {distance:.2f}m"
            cv2.putText(frame, distance_text, (text_x, text_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # 마커 크기 텍스트
            size_text = f"Size: {marker_length*100:.2f}cm"
            cv2.putText(frame, size_text, (text_x, text_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # 거부된 마커 시각화 (디버깅용)
    if rejected:
        cv2.aruco.drawDetectedMarkers(frame, rejected, borderColor=(100, 0, 240))
        # 거부된 마커는 회색 선으로 표시됩니다.

    # 화면 출력
    cv2.imshow('ArUco Marker Pose', frame)

    # ESC 키로 종료
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
