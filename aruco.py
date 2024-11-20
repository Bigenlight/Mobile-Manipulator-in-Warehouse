import cv2
import numpy as np

if __name__ == "__main__":
    # 카메라 캘리브레이션 정보
    camera_matrix = np.array([
        [1.39033663e+03, 0.00000000e+00, 9.91162822e+02],
        [0.00000000e+00, 1.30813689e+03, 3.77377837e+02],
        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
    ])
    dist_coeffs = np.array([-0.63828017, 1.24702716, 0.01994982, 0.01576975, -1.00601758])

    # 마커 길이 (미터 단위, 검은 테두리 포함)
    marker_length = 0.18  # 18cm

    # ArUco 사전 생성
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    # 카메라 초기화
    cap = cv2.VideoCapture(0)  # 2번 카메라 열기

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

            # ID 11과 ID 10의 상대 위치 계산
            if len(ids) >= 2:  # 최소 두 개의 마커가 검출되어야 함
                try:
                    # ID 11의 인덱스
                    idx_11 = np.where(ids == 11)[0][0]
                    # ID 10의 인덱스
                    idx_10 = np.where(ids == 10)[0][0]

                    # ID 11의 회전 및 이동 정보
                    rvec_11, tvec_11 = rvecs[idx_11], tvecs[idx_11]
                    R_11, _ = cv2.Rodrigues(rvec_11)  # 회전 행렬로 변환

                    # ID 10의 회전 및 이동 정보
                    rvec_10, tvec_10 = rvecs[idx_10], tvecs[idx_10]
                    R_10, _ = cv2.Rodrigues(rvec_10)  # 회전 행렬로 변환

                    # ID 11 기준에서 ID 10의 상대 위치
                    relative_position = np.dot(R_11.T, (tvec_10 - tvec_11).reshape(-1, 1)).flatten()
                    print(f"ID 11 기준으로 ID 10의 상대 위치: {relative_position}")

                except IndexError:
                    print("ID 11 또는 ID 10이 검출되지 않았습니다.")

            for i in range(len(ids)):
                # 각 마커의 축 그리기
                cv2.drawFrameAxes(img, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_length * 0.5)

                # 마커 ID 표시
                cv2.putText(img, f"ID: {ids[i][0]}", (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 결과 이미지 표시
        cv2.imshow("ArUco Marker Detection", img)

        # 'q' 버튼을 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 자원 해제
    cap.release()
    cv2.destroyAllWindows()
