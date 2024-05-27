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
    8: "A1", 9: "A2", 
    11: "B1", 12: "B2", 
    14: "C1", 15: "C2", 
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

        # Declare parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('length', 480)
        self.declare_parameter("marker_id", "O1")
        self.declare_parameter("marker_shape", "DICT_4X4_50")
        self.declare_parameter('cam_matrix', [299.26361032, 0.0, 324.93723462, 0.0, 303.22330886, 177.3524136, 0.0, 0.0, 1.0])
        self.declare_parameter('dist_coeff', [0.11564661, -0.05059582, 0.00192533, -0.01093668, -0.03163341])
        self.declare_parameter('left_pid_kp', 1.5)
        self.declare_parameter('left_pid_ki', 0.5)
        self.declare_parameter('left_pid_kd', 1.0)
        self.declare_parameter('right_pid_kp', 1.5)
        self.declare_parameter('right_pid_ki', 0.5)
        self.declare_parameter('right_pid_kd', 1.0)

        self.matrix_coefficients = np.array(self.get_parameter('cam_matrix').get_parameter_value().double_array_value).reshape((3, 3))
        self.distortion_coefficients = np.array(self.get_parameter('dist_coeff').get_parameter_value().double_array_value)

        self.marker_id = self.get_parameter("marker_id").get_parameter_value().string_value
        self.marker_shape = self.get_parameter("marker_shape").get_parameter_value().string_value

        left_kp = self.get_parameter('left_pid_kp').get_parameter_value().double_value
        left_ki = self.get_parameter('left_pid_ki').get_parameter_value().double_value
        left_kd = self.get_parameter('left_pid_kd').get_parameter_value().double_value
        right_kp = self.get_parameter('right_pid_kp').get_parameter_value().double_value
        right_ki = self.get_parameter('right_pid_ki').get_parameter_value().double_value
        right_kd = self.get_parameter('right_pid_kd').get_parameter_value().double_value

        self.left_pid = PIDController(kp=left_kp, ki=left_ki, kd=left_kd)
        self.right_pid = PIDController(kp=right_kp, ki=right_ki, kd=right_kd)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            self.aruco_dict = aruco.getPredefinedDictionary(aruco.__getattribute__(self.marker_shape))
            parameters = aruco.DetectorParameters()
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=parameters)

            p = Pose()
            if ids is not None:
                for i in range(len(ids)):
                    marker_id = ids[i][0]
                    marker_name = marker_id_map.get(marker_id, "Unknown")

                    # Only detect marker id "O1"
                    if marker_name == self.marker_id:
                        self.get_logger().info(f'Marker detected: {marker_name}')
                        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.1, self.matrix_coefficients, self.distortion_coefficients)
                        cv2.drawFrameAxes(cv_image, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec, 0.1)  # The last parameter scales the axis size

                        # position information
                        p.position.x = tvec[0][0][0]
                        p.position.y = tvec[0][0][1]
                        p.position.z = tvec[0][0][2]
                        p.orientation.x = 0.0
                        p.orientation.y = 0.0
                        p.orientation.z = 0.0
                        p.orientation.w = 1.0

                        cv2.polylines(cv_image, [corners[i].astype(np.int32)], True, (0, 0, 255), 3)
                        cv_image = cv2.putText(cv_image, marker_name, (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
                        
                        # Calculate the distance directly from tvec
                        distance = tvec[0][0][2]
                        
                        scaling_factor = 0.2  
                        distance_corrected = distance * scaling_factor

                        # X offset from the camera center
                        x_offset = tvec[0][0][0]

                        # Calculate the ratio of x_offset to distance
                        x_ratio = x_offset / distance_corrected if distance_corrected != 0 else 0

                        left_adjustment = self.left_pid.compute(x_ratio)
                        right_adjustment = self.right_pid.compute(x_ratio)

                        # Check if the marker is directly in front and stop the robot
                        if -0.012 < x_ratio < 0.012:
                            self.arduino.update_command(detected=False, left_wheel_speed=254, right_wheel_speed=0)
                            cv_image = cv2.putText(cv_image, "Front", (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 20), 
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
                        else:
                            if distance_corrected > 0.12:
                                if x_ratio > 0.012:
                                    # Move left
                                    left_wheel_speed = 253 
                                    right_wheel_speed = 0
                                elif x_ratio < -0.012:
                                    # Move right
                                    left_wheel_speed = 254 
                                    right_wheel_speed = 2
                                else:
                                    # Move straight
                                    left_wheel_speed = 254 - left_adjustment
                                    right_wheel_speed = 0 + right_adjustment

                                self.arduino.update_command(detected=True, left_wheel_speed=left_wheel_speed, right_wheel_speed=right_wheel_speed)
                            else:
                                # Stop the robot if the distance is less than or equal to 0.12 meters
                                self.arduino.update_command(detected=False, left_wheel_speed=255, right_wheel_speed=255)

                        if x_ratio > 0.012:
                            cv_image = cv2.arrowedLine(cv_image, (490, 240), (590, 240), (138, 43, 226), 3)
                        elif x_ratio < -0.012:
                            cv_image = cv2.arrowedLine(cv_image, (150, 240), (50, 240), (138, 43, 226), 3)

                        cv_image = cv2.putText(cv_image, f'Distance from AR: {distance_corrected:.2f}m, {self.marker_id}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
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
