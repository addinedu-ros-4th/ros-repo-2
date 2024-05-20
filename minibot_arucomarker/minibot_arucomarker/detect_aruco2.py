import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from cv2 import aruco
from geometry_msgs.msg import Pose
import os
import serial

marker_id_map = {
    0: "I1", 1: "I2", 2: "I3",
    3: "O1", 4: "O2", 5: "O3",
    6: "R1", 7: "R2",
    8: "A1", 9: "A2", 10: "A3",
    11: "B1", 12: "B2", 13: "B3",
    14: "C1", 15: "C2", 16: "C3",
    17: "P1", 18: "P2", 19: "P3"
}

class ArduinoController:
    def __init__(self, port="/dev/ttyArduino", baudrate=1000000):
        self.ser = serial.Serial(port, baudrate)
        self.command = [0xfa, 0xfe, 2, 0, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0xfa, 0xfd]
        self.update_command(detected=0)

    def send_initial_commands(self):
        initial_commands = [
            b"\xfa\xfe\x03\x01\x04\xfa\xfd",
            b"\xfa\xfe\x01\x01\x01\x03\x06\xfa\xfd"
        ]
        for command in initial_commands:
            self.ser.write(command)
            print(self.read(size=20, timeout=1))

    def read(self, size=1, timeout=None):
        self.ser.timeout = timeout
        return self.ser.read(size)

    def update_command(self, detected, left_wheel_speed=255, right_wheel_speed=255):
        self.command[3] = 1 if detected else 0
        self.command[4] = int(left_wheel_speed)  # Cast to int
        self.command[6] = int(right_wheel_speed)  # Cast to int
        self.command[12] = np.uint8(sum(self.command[2:12]))
        self.ser.write(bytes(self.command))

    def __del__(self):
        if self.ser.is_open:
            self.ser.close()


class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def compute(self, current_value):
        error = self.setpoint - current_value
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class ImageConverter(Node):
    def __init__(self):
        super().__init__('image_converter')

        self.pose_pub = self.create_publisher(Pose, 'pose', 1)
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera', self.callback, 1)

        # Initialize ArduinoController
        self.arduino = ArduinoController()
        self.arduino.send_initial_commands()

        # Load calibration data
        npz_path = '/home/yongtak_ras/pinkbot/src/pinklab_minibot_robot/minibot_arucomarker/config/MultiMatrix.npz'
        if not os.path.exists(npz_path):
            self.get_logger().error(f'Calibration file not found: {npz_path}')
            raise FileNotFoundError(f'Calibration file not found: {npz_path}')
        with np.load(npz_path) as data:
            self.matrix_coefficients = data['camMatrix']
            self.distortion_coefficients = data['distCoef']

        # Declare parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('length', 480)

        # Initialize PID controllers with tuned gains
        self.left_pid = PIDController(kp=2.0, ki=0.1, kd=1.0)
        self.right_pid = PIDController(kp=2.0, ki=0.1, kd=1.0)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            parameters = aruco.DetectorParameters()
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            p = Pose()
            if ids is not None:
                for i in range(len(ids)):
                    marker_id = ids[i][0]
                    marker_name = marker_id_map.get(marker_id, "Unknown")

                    # Only process the marker if it is "O1"
                    if marker_name == "O1":
                        self.get_logger().info(f'Marker detected: {marker_name}')
                        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.1, self.matrix_coefficients, self.distortion_coefficients)
                        cv2.drawFrameAxes(cv_image, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec, 0.1)  # The last parameter scales the axis size

                        # Directly use tvec for position information
                        p.position.x = tvec[0][0][0]
                        p.position.y = tvec[0][0][1]
                        p.position.z = tvec[0][0][2]
                        p.orientation.x = 0.0
                        p.orientation.y = 0.0
                        p.orientation.z = 0.0
                        p.orientation.w = 1.0

                        # Draw marker name on the image
                        cv2.polylines(cv_image, [corners[i].astype(np.int32)], True, (0, 0, 255), 3)
                        cv_image = cv2.putText(cv_image, marker_name, (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
                        
                        # Calculate the distance directly from tvec
                        distance = tvec[0][0][2]
                        
                        scaling_factor = 0.5  # Adjust this factor if needed
                        distance_corrected = distance * scaling_factor

                        # X offset from the camera center
                        x_offset = tvec[0][0][0]

                        # Calculate the ratio of x_offset to distance
                        x_ratio = x_offset / distance_corrected if distance_corrected != 0 else 0

                        # Check if the marker is directly in front and stop the robot
                        if -0.012 < x_ratio < 0.012:
                            self.arduino.update_command(detected=False, left_wheel_speed=252, right_wheel_speed=2)
                            cv_image = cv2.putText(cv_image, "Front", (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 20), 
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
                        else:
                            # Movement logic based on x_ratio
                            if distance_corrected > 0.12:
                                if x_ratio > 0.012:
                                    # Move left
                                    left_wheel_speed = 252
                                    right_wheel_speed = 0
                                elif x_ratio < -0.012:
                                    # Move right
                                    left_wheel_speed = 254
                                    right_wheel_speed = 3
                                else:
                                    # Move straight
                                    left_wheel_speed = 252
                                    right_wheel_speed = 2

                                self.arduino.update_command(detected=True, left_wheel_speed=left_wheel_speed, right_wheel_speed=right_wheel_speed)
                            else:
                                # Stop the robot if the distance is less than or equal to 0.12 meters
                                self.arduino.update_command(detected=False, left_wheel_speed=255, right_wheel_speed=255)

                        # Draw arrows based on the marker's position
                        if x_ratio > 0.012:
                            cv_image = cv2.arrowedLine(cv_image, (490, 240), (590, 240), (138, 43, 226), 3)
                        elif x_ratio < -0.012:
                            cv_image = cv2.arrowedLine(cv_image, (150, 240), (50, 240), (138, 43, 226), 3)

                        # Display corrected distances on the image
                        cv_image = cv2.putText(cv_image, f'Distance: {distance_corrected:.2f}m', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
            else:
                self.arduino.update_command(detected=False)

            width = self.get_parameter('width').get_parameter_value().integer_value
            length = self.get_parameter('length').get_parameter_value().integer_value
            cv_image = cv2.resize(cv_image, (width, length))
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(1)

            self.pose_pub.publish(p)

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')


def main(args=None):
    rclpy.init(args=args)
    ic = ImageConverter()
    try:
        rclpy.spin(ic)
    except KeyboardInterrupt:
        ic.get_logger().info('Shutting down')
    ic.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
