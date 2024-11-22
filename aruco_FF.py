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
    marker_length = 0.107  # 10.7cm

    # ArUco 사전 생성
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    # 카메라 초기화 (기본 카메라 0번)
    cap = cv2.VideoCapture(4)

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

            # ID와 인덱스 매핑 생성
            marker_indices = {id[0]: idx for idx, id in enumerate(ids)}

            # 기준 마커 ID41이 검출되었는지 확인
            if 41 in marker_indices:
                ref_idx = marker_indices[41]
                rvec_ref = rvecs[ref_idx]
                tvec_ref = tvecs[ref_idx]

                # 기준 마커의 변환 행렬 계산
                R_ref, _ = cv2.Rodrigues(rvec_ref)
                T_ref_to_cam = np.eye(4)
                T_ref_to_cam[:3,:3] = R_ref
                T_ref_to_cam[:3,3] = tvec_ref.reshape(3)

                # 카메라로부터 기준 마커로의 변환 계산
                T_cam_to_ref = np.linalg.inv(T_ref_to_cam)

                # 다른 마커들에 대해 반복
                for target_id in [40, 42]:
                    if target_id in marker_indices:
                        target_idx = marker_indices[target_id]
                        rvec_target = rvecs[target_idx]
                        tvec_target = tvecs[target_idx]

                        # 타겟 마커의 변환 행렬 계산
                        R_target, _ = cv2.Rodrigues(rvec_target)
                        T_target_to_cam = np.eye(4)
                        T_target_to_cam[:3,:3] = R_target
                        T_target_to_cam[:3,3] = tvec_target.reshape(3)

                        # 타겟 마커의 기준 마커에 대한 변환 계산
                        T_target_to_ref = T_cam_to_ref @ T_target_to_cam

                        # 상대 위치 추출
                        t_target_to_ref = T_target_to_ref[:3,3]

                        # 거리 계산
                        distance = np.linalg.norm(t_target_to_ref)

                        # 각도 계산 (XY 평면에서의 각도)
                        angle = np.degrees(np.arctan2(t_target_to_ref[1], t_target_to_ref[0]))

                        # 텍스트 표시를 위한 좌표 계산 (타겟 마커의 좌상단 코너 사용)
                        corner = corners[target_idx][0][0]
                        text_position = (int(corner[0]), int(corner[1]) - 30)

                        # 상대 좌표 및 거리, 각도 표시
                        coord_text = f"Rel X: {t_target_to_ref[0]:.2f}m Y: {t_target_to_ref[1]:.2f}m Z: {t_target_to_ref[2]:.2f}m"
                        distance_text = f"Distance to ID41: {distance:.2f}m"
                        angle_text = f"Angle to ID41: {angle:.2f} deg"

                        cv2.putText(img, coord_text, text_position,
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        cv2.putText(img, distance_text, (text_position[0], text_position[1] + 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        cv2.putText(img, angle_text, (text_position[0], text_position[1] + 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                # 각 마커의 축 그리기
                for i in range(len(ids)):
                    cv2.drawFrameAxes(img, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_length * 0.5)

                    # 마커 ID 표시
                    corner = corners[i][0][0]
                    cv2.putText(img, f"ID: {ids[i][0]}", (int(corner[0]), int(corner[1]) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # 카메라로부터의 거리 계산
                    tvec = tvecs[i][0]
                    distance = np.linalg.norm(tvec)

                    # 좌표 및 거리 표시
                    coord_text = f"X: {tvec[0]:.2f}m Y: {tvec[1]:.2f}m Z: {tvec[2]:.2f}m"
                    distance_text = f"Distance: {distance:.2f}m"

                    cv2.putText(img, coord_text, (int(corner[0]), int(corner[1]) + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    cv2.putText(img, distance_text, (int(corner[0]), int(corner[1]) + 35),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            else:
                # 기준 마커가 검출되지 않은 경우, 기존의 마커 표시를 그대로 진행
                for i in range(len(ids)):
                    # 각 마커의 축 그리기
                    cv2.drawFrameAxes(img, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_length * 0.5)

                    # 마커 ID 표시
                    corner = corners[i][0][0]
                    cv2.putText(img, f"ID: {ids[i][0]}", (int(corner[0]), int(corner[1]) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # 카메라로부터의 거리 계산
                    tvec = tvecs[i][0]
                    distance = np.linalg.norm(tvec)

                    # 좌표 및 거리 표시
                    coord_text = f"X: {tvec[0]:.2f}m Y: {tvec[1]:.2f}m Z: {tvec[2]:.2f}m"
                    distance_text = f"Distance: {distance:.2f}m"

                    cv2.putText(img, coord_text, (int(corner[0]), int(corner[1]) + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    cv2.putText(img, distance_text, (int(corner[0]), int(corner[1]) + 35),
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
