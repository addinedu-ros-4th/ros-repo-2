import cv2

# 카메라 인덱스 또는 디바이스 경로 사용
cap = cv2.VideoCapture('/dev/video0')  # 0은 보통 첫 번째 연결된 카메라를 의미

if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 받아올 수 없습니다. 종료합니다.")
        break

    # 여기에서 프레임 처리
    cv2.imshow('Video', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()