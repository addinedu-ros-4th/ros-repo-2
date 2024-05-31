import cv2 as cv
import os

# Chessboard dimensions
Chess_Board_Dimensions = (9, 6)
n = 0  # image counter

# Image path
image_path = "/home/yongtak_ras/test/picamera/camera_calibaration/images"

# Checks if images directory exists or not
Dir_Check = os.path.isdir(image_path)

if not Dir_Check:  # If directory does not exist, create it
    os.makedirs(image_path)
    print(f'"{image_path}" Directory is created')
else:
    print(f'"{image_path}" Directory already exists.')

# Criteria for corner refinement
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

def detect_checker_board(image, grayImage, criteria, boardDimension):
    ret, corners = cv.findChessboardCorners(grayImage, boardDimension)
    if ret:
        corners1 = cv.cornerSubPix(grayImage, corners, (3, 3), (-1, -1), criteria)
        image = cv.drawChessboardCorners(image, boardDimension, corners1, ret)
    return image, ret

# Initialize video capture
cap = cv.VideoCapture('/dev/video0')

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    copyFrame = frame.copy()
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    image, board_detected = detect_checker_board(frame, gray, criteria, Chess_Board_Dimensions)
    
    if board_detected:
        print("Chessboard detected")

    # Display the number of saved images
    cv.putText(frame, f"saved_img : {n}", (30, 40), cv.FONT_HERSHEY_PLAIN, 1.4, (0, 255, 0), 2, cv.LINE_AA)

    # Show the frames
    cv.imshow("frame", frame)
    cv.imshow("copyFrame", copyFrame)

    key = cv.waitKey(1)

    if key == ord("s"):
        break
    if key == ord("q"):
        print("1 key pressed")
        if board_detected:
            # Save the image if '1' is pressed and chessboard is detected
            cv.imwrite(f"{image_path}/image{n}.png", copyFrame)
            print(f"saved image number {n}")
            n += 1  # Increment image counter
        else:
            print("Chessboard not detected, image not saved")

# Release resources
cap.release()
cv.destroyAllWindows()

print("Total saved Images:", n)