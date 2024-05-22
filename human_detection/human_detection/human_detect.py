import time
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_proto_path', '/human_detection/models/MobileNetSSD_deploy.prototxt'),
                ('model_weights_path', '/human_detection/models/MobileNetSSD_deploy.caffemodel'),
                ('conf_threshold', 0.2),
                ('frame_width', 320),
                ('frame_height', 240),
                ('fps', 30),
                ('detection_interval', 0.5)  # 0.5초마다 감지 수행
            ]
        )
        model_proto_path = self.get_parameter('model_proto_path').get_parameter_value().string_value
        model_weights_path = self.get_parameter('model_weights_path').get_parameter_value().string_value

        # 상대 경로를 절대 경로로 변환
        model_proto_path = os.path.abspath(model_proto_path)
        model_weights_path = os.path.abspath(model_weights_path)

        self.net = cv2.dnn.readNetFromCaffe(
            model_proto_path,
            model_weights_path
        )

        self.publisher_ = self.create_publisher(String, 'detect', 10)
        self.timer = self.create_timer(1.0 / self.get_parameter('fps').get_parameter_value().integer_value, self.timer_callback)

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.get_parameter('frame_width').get_parameter_value().integer_value)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.get_parameter('frame_height').get_parameter_value().integer_value)
        self.cap.set(cv2.CAP_PROP_FPS, self.get_parameter('fps').get_parameter_value().integer_value)

        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.classNames = {15: 'person'}
        self.last_detection_time = time.time()

    def timer_callback(self):
        ret, image = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to read frame")
            return

        now = time.time()
        if now - self.last_detection_time < self.get_parameter('detection_interval').get_parameter_value().double_value:
            return

        self.last_detection_time = now

        (h, w) = image.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)),
                                     0.007843, (300, 300), 127.5)
        self.net.setInput(blob)
        detections = self.net.forward()

        detected = False
        for i in np.arange(0, detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > self.conf_threshold:
                idx = int(detections[0, 0, i, 1])
                if idx in self.classNames:
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype("int")
                    label = f"{self.classNames[idx]}: {confidence:.2f}"

                    cv2.rectangle(image, (startX, startY), (endX, endY), (255, 0, 0), 2)
                    y = startY - 15 if startY - 15 > 15 else startY + 15
                    cv2.putText(image, label, (startX, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                    self.get_logger().info(f"Person detected: {confidence:.2f}")
                    msg = String()
                    msg.data = "Person detected"
                    self.publisher_.publish(msg)
                    detected = True

        # cv2.imshow("Frame", image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    person_detector = PersonDetector()
    rclpy.spin(person_detector)
    person_detector.destroy_node()
    rclpy.shutdown()
    person_detector.cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
