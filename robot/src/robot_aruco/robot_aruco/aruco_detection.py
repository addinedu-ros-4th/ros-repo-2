import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from cv2 import aruco
import time
from task_msgs.srv import ArucoCommand
from task_msgs.srv import ArucoCommandResponse
from geometry_msgs.msg import Twist
class RobotAruco(Node):
    def __init__(self):
        super().__init__('image_converter')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(CompressedImage, '/camera/compressed', self.aruco_callback, 10)
        self.subscription # prevent unused variable warning
        self.server = self.create_service(ArucoCommand, '/aruco_control', self.handle_aruco_control)
        self.cmd_pub = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.marker_done_client = self.create_client(
            ArucoCommandResponse, "/aruco_command_response"
        )
        self.twist = Twist()
        self.marker_name = None
        self.location = None
        self.distance = None
        self.x_offset = None
        self.tvec = None
        self.aruco_toggle = False
        self.marker_id_map = {
            0: "I1", 1: "I2", 2: "I3",
            3: "O1", 4: "O2", 5: "O3",
            6: "R1", 7: "R2",
            8: "A1", 9: "A2",
            11: "B1", 12: "B2",
            14: "C1", 15: "C2",
            17: "P1", 18: "P2", 19: "P3"
        }
        # Declare parameters
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter("marker_shape", "DICT_4X4_50")
        self.declare_parameter('cam_matrix', [215.19041944, 0.0, 184.50578309, 0.0, 219.25108305, 80.0767011, 0.0, 0.0, 1.0])
        self.declare_parameter('dist_coeff', [0.29089896, -0.61490299, 0.04594559, 0.06252445, 0.65154033])
        # Define parameters
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.marker_shape = self.get_parameter('marker_shape').value
        self.matrix_coefficients = np.array(self.get_parameter('cam_matrix').get_parameter_value().double_array_value).reshape((3, 3))
        self.distortion_coefficients = np.array(self.get_parameter('dist_coeff').get_parameter_value().double_array_value)
        # Initialize aruco_dict
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.__getattribute__(self.marker_shape))
    def handle_aruco_control(self, request, response):
        self.location = request.location
        self.direction = request.direction
        self.aruco_toggle = True
        self.get_logger().info(f"start picking place & floor {request.location}, {request.direction}")
        return response
    def send_response(self):
        res = ArucoCommandResponse.Request()
        res.success = True
        self.marker_done_client.call_async(res)
    def motor_control(self):
        self.distance = self.tvec[0][0][2]
        self.x_offset = self.tvec[0][0][0]
        self.get_logger().info(f"Distance: {self.distance}, X offset: {self.x_offset}")
        if self.direction == 'forward':
            if self.distance <= 0.115:
                # self.get_logger().info("Marker within stopping distance, robot stopped.")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
                self.send_response()
                self.aruco_toggle = False
            else:
                if -0.15 < self.x_offset < 0.15:
                    if -0.02 < self.x_offset < 0.02:
                        # self.get_logger().info("moving straight.")
                        self.twist.linear.x = 0.12  # Move forward
                        self.twist.angular.z = 0.0
                    elif self.x_offset > 0.02:
                        # self.get_logger().info("turning left.")
                        self.twist.linear.x = 0.12 # Stop forward movement
                        self.twist.angular.z = -0.04  # Turn left
                    elif self.x_offset < -0.02:
                        # self.get_logger().info("turning right.")
                        self.twist.linear.x = 0.12  # Stop forward movement
                        self.twist.angular.z = 0.04  # Turn right
                else:
                    if self.x_offset > 0.02:
                        # self.get_logger().info("turning left.")
                        self.twist.linear.x = 0.12  # Stop forward movement
                        self.twist.angular.z = -0.06  # Turn left
                    elif self.x_offset < -0.02:
                        # self.get_logger().info("turning right.")
                        self.twist.linear.x = 0.12  # Stop forward movement
                        self.twist.angular.z = 0.06  # Turn right
                self.cmd_pub.publish(self.twist)
        elif self.direction == 'backward':
            if self.distance >= 0.19:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
                self.send_response()
                self.aruco_toggle = False
            else:
                # self.get_logger().info("Marker directly in front, moving backward.")
                self.twist.linear.x = -0.05
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
    def aruco_callback(self, data):
        np_arr = np.frombuffer(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        parameters = aruco.DetectorParameters()
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=parameters)
        if ids is not None:
            # self.get_logger().info(f'Detected {len(ids)} markers.')
            for i in range(len(ids)):
                marker_id = ids[i][0]
                marker_name = self.marker_id_map.get(marker_id, "Unknown")
                self.get_logger().info(f'Marker detected: {marker_name}')
                self.rvec, self.tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.025, self.matrix_coefficients, self.distortion_coefficients)
                cv_image = cv2.drawFrameAxes(cv_image, self.matrix_coefficients, self.distortion_coefficients, self.rvec, self.tvec, 0.1)
                if self.aruco_toggle and (self.location == marker_name):
                    self.motor_control()
        else:
            pass
            # self.get_logger().info('No markers detected.')

        cv2.imshow('Aruco Detection', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    ic = RobotAruco()
    try:
        rclpy.spin(ic)
    except KeyboardInterrupt:
        ic.get_logger().info('Shutting down')
    ic.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
if __name__ == '__main__':
    main()