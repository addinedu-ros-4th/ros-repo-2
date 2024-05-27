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
import RPi.GPIO as GPIO
import time
import smbus2 as smbus
from std_srvs.srv import SetBool


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
        command = b"\xfa\xfe\x01\x01\x01\x03\x06\xfa\xfd"
        self.ser.write(command)
        print(self.read(size=20, timeout=1))

    def read(self, size=1, timeout=None):
        self.ser.timeout = timeout
        return self.ser.read(size)

    def update_command(self, detected, left_wheel_speed=0, right_wheel_speed=0):
        self.command[3] = 1 if detected else 0
        self.command[4] = int(left_wheel_speed)  # Cast to int
        self.command[6] = int(right_wheel_speed)  # Cast to int
        self.command[12] = np.uint8(sum(self.command[2:12]))
        self.ser.write(bytes(self.command))

    def __del__(self):
        if self.ser.is_open:
            self.ser.close()

# 추가
class RasGPIOController:
    def __init__(self, step_pins):
        # pin 번호 설정
        self.step_pins = step_pins
        
        # GPIO 설정
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        for pin in self.step_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, False)

        # floor 설정 
        self.current_step = -6000
        self.stepfloors = {"1place": 0, "1lift": -3000, "2place": -6000, "2lift": -7000}

    def step_controll(self, floor):
        desired_step  = self.stepfloors[floor]
        step_diff = desired_step - self.current_step 
         
        StepCount = 4

        Seq = [[1, 0, 0, 0],
               [0, 1, 0, 0],
               [0, 0, 1, 0],
               [0, 0, 0, 1]]
        
        StepCounter = 0
        direction = 1 if step_diff > 0 else -1
        step_diff = abs(step_diff)
        WaitTime = 0.002

        for _ in range(step_diff):
            for pin in range(0, 4):
                xpin = self.step_pins[pin]
                if Seq[StepCounter][pin] != 0:
                    GPIO.output(xpin, True)
                else:
                    GPIO.output(xpin, False)
            StepCounter += direction
            if StepCounter >= StepCount:
                StepCounter = 0
            if StepCounter < 0:
                StepCounter = StepCount - 1
            time.sleep(WaitTime)
        
        self.current_step = desired_step


class ImageConverter(Node):
    def __init__(self):
        super().__init__('image_converter')

        self.pose_pub = self.create_publisher(Pose, 'pose', 1)
        self.bridge = CvBridge()
        self.image_sub = None  

        # Initialize ArduinoController & RasController
        self.arduino = ArduinoController()
        self.arduino.send_initial_commands()
        self.rascontroller = RasGPIOController([17, 18, 22, 23])

        # Declare parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter("marker_id", "O1")
        self.declare_parameter("marker_shape", "DICT_4X4_50")
        self.declare_parameter('cam_matrix', [299.26361032, 0.0, 324.93723462, 0.0, 303.22330886, 177.3524136, 0.0, 0.0, 1.0])
        self.declare_parameter('dist_coeff', [0.11564661, -0.05059582, 0.00192533, -0.01093668, -0.03163341])
        
        # Define parameters
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.marker_id = self.get_parameter('marker_id').value
        self.marker_shape = self.get_parameter('marker_shape').value
        self.matrix_coefficients = np.array(self.get_parameter('cam_matrix').get_parameter_value().double_array_value).reshape((3, 3))
        self.distortion_coefficients = np.array(self.get_parameter('dist_coeff').get_parameter_value().double_array_value)

        # Initialize aruco_dict
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.__getattribute__(self.marker_shape))


    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            parameters = aruco.DetectorParameters()
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=parameters)
            # cv2.imshow("Image window", cv_image)

            # p = Pose()
            if ids is not None:
                for i in range(len(ids)):
                    marker_id = ids[i][0]
                    marker_name = marker_id_map.get(marker_id, "Unknown")

                    # Only detect marker id "O1"
                    if marker_name == self.marker_id:
                        self.get_logger().info(f'Marker detected: {marker_name}')
                        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.025, self.matrix_coefficients, self.distortion_coefficients)
                        cv2.drawFrameAxes(cv_image, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec, 0.1)
                        
                        # position information
                        # p.position.x = tvec[0][0][0]
                        # p.position.y = tvec[0][0][1]
                        # p.position.z = tvec[0][0][2]
                        # p.orientation.x = 0.0
                        # p.orientation.y = 0.0
                        # p.orientation.z = 0.0
                        # p.orientation.w = 1.0

                        distance = tvec[0][0][2]
                        x_offset = tvec[0][0][0]

                        # Check if the marker is directly in front and stop the robot
                        if distance <= 0.08:
                            self.arduino.update_command(detected=False, left_wheel_speed=0, right_wheel_speed=0)
                            self.get_logger().info("Marker within stopping distance, robot stopped.")
                            time.sleep(0.2)
                            self.rascontroller.step_controll("2lift")
                        else:
                            if -0.15 < x_offset < 0.15:
                                if -0.05 < x_offset < 0.05:
                                    self.get_logger().info("Marker directly in front, moving straight.")
                                    left_wheel_speed = 254
                                    right_wheel_speed = 2
                                elif x_offset > 0.05:
                                    self.get_logger().info("Marker slightly to the right, slight left adjustment.")
                                    left_wheel_speed = 253
                                    right_wheel_speed = 2
                                elif x_offset < -0.05:
                                    self.get_logger().info("Marker slightly to the left, slight right adjustment.")
                                    left_wheel_speed = 254
                                    right_wheel_speed = 3
                            else:
                                if x_offset > 0.15:
                                    self.get_logger().info("Marker to the right, turning left.")
                                    left_wheel_speed = 253 
                                    right_wheel_speed = 0
                                elif x_offset < -0.15:
                                    self.get_logger().info("Marker to the left, turning right.")
                                    left_wheel_speed = 0 
                                    right_wheel_speed = 3

                            self.arduino.update_command(detected=True, left_wheel_speed=left_wheel_speed, right_wheel_speed=right_wheel_speed)
            else:
                # 감지 실패
                self.arduino.update_command(detected=False)

            # cv_image = cv2.resize(cv_image, (self.width, self.height))
            # cv2.waitKey(1)

            # self.pose_pub.publish(p)

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in callback: {e}')


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
