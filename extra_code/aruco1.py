import cv2
import numpy as np

def main():
    # 카메라 캘리브레이션 정보
    camera_matrix = np.array([
        [4.20206927e+02, 0.00000000e+00, 3.09996813e+02],
        [0.00000000e+00, 4.22554304e+02, 2.32743975e+02],
        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
    ])
    
    dist_coeffs = np.array([5.96212387e-02, -1.55974578e-01, -9.76032365e-03, 2.33071430e-03, 1.74747614e-01])

    # 마커 길이 (미터 단위, 검은 테두리 포함)
    marker_length = 0.107  # 10.5cm

    # ArUco 사전 생성
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    # 카메라 초기화 (기본 카메라 0번)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        exit()

    print("카메라 초기화 완료: 'q' 버튼을 누르면 종료됩니다.")

    while True:
        # 카메라에서 이미지 읽기
        ret, img = cap.read()
        if not ret:
            print("이미지를 가져올 수 없습니다.")
            break

        # 그레이스케일 변환
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 마커 검출
        corners, ids, rejectedCandidates = detector.detectMarkers(gray)

        # 마커가 검출된 경우
        if ids is not None:
            # 마커의 포즈 추정
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

            for i in range(len(ids)):
                # 각 마커의 축 그리기
                cv2.drawFrameAxes(img, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_length * 0.5)

                # 마커 ID 표시
                # 마커의 좌상단 코너에 ID를 표시
                corner = corners[i][0][0]
                cv2.putText(img, f"ID: {ids[i][0]}", (int(corner[0]), int(corner[1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # 마커의 3D 좌표와 거리 계산
                tvec = tvecs[i][0]
                distance = np.linalg.norm(tvec)  # 카메라로부터의 거리

                # 좌표 및 거리 표시
                coord_text = f"X: {tvec[0]:.2f}m Y: {tvec[1]:.2f}m Z: {tvec[2]:.2f}m"
                distance_text = f"Distance: {distance:.2f}m"

                # 텍스트를 마커 근처에 표시
                text_position = (int(corner[0]), int(corner[1]) + 20)
                cv2.putText(img, coord_text, text_position,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                cv2.putText(img, distance_text, (text_position[0], text_position[1] + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # 결과 이미지 표시
        cv2.imshow("ArUco Marker Detection", img)

        # 'q' 버튼을 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 자원 해제
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
