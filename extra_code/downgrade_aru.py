import cv2
import numpy as np

# Load camera calibration parameters
calibration_path = '/home/theo/3_ws/src'
camera_matrix = np.load(f'{calibration_path}/calibration_matrix_webcam1.npy')
dist_coeffs = np.load(f'{calibration_path}/distortion_coefficients_webcam1.npy')

# Select ArUco dictionary
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)

# Create DetectorParameters
aruco_params = cv2.aruco.DetectorParameters_create()

# Real size of the ArUco marker (in meters)
marker_length = 0.107  # Update this value based on your marker's actual size

# Define the marker IDs you want to detect
target_marker_ids = [41, 42]

# Start capturing from webcam
cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

print("카메라가 열렸습니다. ArUco 마커(ID41, ID42)를 카메라에 보여주세요.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("카메라 프레임을 읽을 수 없습니다.")
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    # Output detection status
    if ids is None:
        print("마커를 감지하지 못했습니다.")
    else:
        detected_ids = ids.flatten()
        print(f"감지된 마커 ID: {detected_ids}")

    # Dictionary to store the translation vectors of detected markers
    marker_positions = {}

    # Visualization and pose estimation
    if ids is not None:
        for i, (corner, marker_id) in enumerate(zip(corners, ids.flatten())):
            if marker_id not in target_marker_ids:
                continue  # Only process markers with ID 41 and 42

            # Pose estimation
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corner, marker_length, camera_matrix, dist_coeffs
            )

            # Pose estimation results are arrays; extract the first marker
            rvec = rvecs[0][0]
            tvec = tvecs[0][0]

            # Store the translation vector in the dictionary
            marker_positions[marker_id] = tvec

            # Output relative coordinates
            x, y, z = tvec
            distance = np.linalg.norm(tvec)
            print(f"Marker ID: {marker_id} | Position: x={x:.3f}, y={y:.3f}, z={z:.3f} | Distance: {distance:.3f}m")

            # Visualization
            cv2.aruco.drawDetectedMarkers(frame, [corner], ids[i:i+1])
            cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_length)

            # Text overlay
            corner_position = tuple(corner[0][0].astype(int))
            text_x, text_y = corner_position

            if text_y - 60 > 10:
                text_y -= 60
            else:
                text_y += 60

            # Relative coordinate text
            coord_text = f"ID:{marker_id} x:{x:.2f} y:{y:.2f} z:{z:.2f}"
            cv2.putText(frame, coord_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Distance text
            distance_text = f"Distance: {distance:.2f}m"
            cv2.putText(frame, distance_text, (text_x, text_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Marker size text
            size_text = f"Size: {marker_length*100:.2f}cm"
            cv2.putText(frame, size_text, (text_x, text_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Visualization of rejected markers (for debugging)
    if rejected:
        cv2.aruco.drawDetectedMarkers(frame, rejected, borderColor=(100, 0, 240))

    # Calculate distance between two markers if both are detected
    if len(marker_positions) == 2:
        # Extract positions
        pos1 = marker_positions[target_marker_ids[0]]
        pos2 = marker_positions[target_marker_ids[1]]

        # Calculate Euclidean distance between the two markers
        inter_marker_distance = np.linalg.norm(pos1 - pos2)
        print(f"Markers {target_marker_ids[0]} and {target_marker_ids[1]} 사이 거리: {inter_marker_distance:.3f}m")

        # Optional: Display the inter-marker distance on the frame
        # Choose a position to place the text, e.g., center of the frame
        frame_center = (frame.shape[1] // 2, frame.shape[0] // 2)
        distance_text = f"Distance between ID41 & ID42: {inter_marker_distance:.2f}m"
        cv2.putText(frame, distance_text, frame_center, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    # Display the frame
    cv2.imshow('ArUco Marker Pose', frame)

    # Exit on ESC key
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
