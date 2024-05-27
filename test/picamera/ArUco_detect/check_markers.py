import cv2 as cv
from cv2 import aruco
import numpy as np

# Dictionary to specify type of the marker
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# Detector parameters
param_markers = aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(marker_dict, param_markers)

# Utilize default camera/webcam driver
cap = cv.VideoCapture('/dev/video0')

# Map detected marker IDs to predetermined IDs
marker_id_map = {
    0: "I1", 1: "I2", 2: "I3",
    3: "O1", 4: "O2", 5: "O3",
    6: "R1", 7: "R2",
    8: "A1", 9: "A2", 10: "A3",
    11: "B1", 12: "B2", 13: "B3",
    14: "C1", 15: "C2", 16: "C3",
    17: "P1", 18: "P2", 19: "P3" 
}

# Iterate through multiple frames, in a live video feed
while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Convert the frame to grayscale (for efficiency)
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    # Detect markers
    marker_corners, marker_IDs, _ = detector.detectMarkers(gray_frame)
    
    # Check if markers are detected
    if marker_corners:
        for ids, corners in zip(marker_IDs, marker_corners):
            # Draw marker outline
            cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
            
            # Reshape corners
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            
            # Get the predetermined ID
            pred_id = marker_id_map.get(ids[0], "Unknown")
            
            # Draw the predetermined marker ID
            top_right = corners[0].ravel()
            cv.putText(
                frame,
                f"id: {pred_id}",
                tuple(top_right),
                cv.FONT_HERSHEY_PLAIN,
                1.3,
                (200, 100, 0),
                2,
                cv.LINE_AA,
            )

    # Display frame
    cv.imshow("frame", frame)
    
    # Check for quit command
    key = cv.waitKey(1)
    if key == ord("q"):
        break
