import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from cv2 import aruco
from task_msgs.srv import ArucoCommand
from task_msgs.srv import ArucoCommandResponse
from nav2_simple_commander.robot_navigator import BasicNavigator
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.action import DriveOnHeading
from rclpy.action import ActionClient
import time

class ArucoCam(Node):
    def __init__(self):
        super().__init__('aruco_cam')
        self.nav = BasicNavigator()
        self.img_pub = self.create_publisher(CompressedImage, '/camera/compressed', 10) 
        self.aruco_server = self.create_service(ArucoCommand, '/aruco_control', self.handle_aruco_control)
        self.cmd_pub = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.marker_done_client = self.create_client(ArucoCommandResponse, "/aruco_command_response")
        self.camera_server = self.create_service(SetBool, '/camera_control', self.handle_camera_control)
        self.timer = self.create_timer(0.01, self.aruco_callback)
        self.drive_on_heading_client = ActionClient(self, DriveOnHeading, 'drive_on_heading')

        # videocapture instance
        self.fps = 25
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cam_toggle = False

        # cv_bridge, img_msg
        self.cv_bridge = CvBridge()
        self.img_msg = CompressedImage()

        # Declare parameters
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter("marker_shape", "DICT_4X4_50")
        self.declare_parameter('cam_matrix', [249.26653787, 0.0, 160.31833382, 0.0, 250.05718578, 114.24750264, 0.0, 0.0, 1.0])
        self.declare_parameter('dist_coeff', [0.166805069, -0.368234503, 0.0000476781203, 0.00262091227, 0.172290440])
        
        # Define parameters
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.marker_shape = self.get_parameter('marker_shape').value
        self.matrix_coefficients = np.array(self.get_parameter('cam_matrix').get_parameter_value().double_array_value).reshape((3, 3))
        self.distortion_coefficients = np.array(self.get_parameter('dist_coeff').get_parameter_value().double_array_value)
        
        # Initialize aruco_dict
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.__getattribute__(self.marker_shape))

        # Initialize Parameters
        self.twist = Twist()
        self.marker_name = None
        self.location = None
        self.distance = None
        self.x_offset = None
        self.tvec = None
        self.is_mid = False
        self.aruco_toggle = False
        self.is_mid = False

        self.target_distance = 0.0
        self.marker_id_map = {
            0: "I1", 1: "I2", 2: "I3",
            3: "O1", 4: "O2", 5: "O3",
            6: "R1", 7: "R2",
            8: "A1", 9: "A2",
            11: "B1", 12: "B2",
            14: "C1", 15: "C2",
            17: "P1", 18: "P2", 19: "P3"
        }
        self.goal_msg = DriveOnHeading.Goal()

    def handle_camera_control(self, request, response):
        self.cam_toggle = request.data
        response.success = True
        response.message = 'Success' if request.data else 'Failed'
        return response


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

    def motor_control(self, pitch):
        self.distance = self.tvec[0][0][2]
        self.x_offset = self.tvec[0][0][0]
        # self.get_logger().info(f"Distance: {self.distance}, X offset: {self.x_offset}")
        
        if self.direction == 'forward':
            if self.distance <= 0.17:
                self.get_logger().info("Marker within stopping distance, robot stopped.")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
                self.send_response()
                self.aruco_toggle = False

                self.is_mid = False
                self.target_distance = 0.0
            else:
                if -0.1 < self.x_offset < 0.1:
                    if -0.02 < self.x_offset < 0.02:
                        self.get_logger().info("moving straight.")
                        self.twist.linear.x = 0.05  # Move forward
                        self.twist.angular.z = 0.0
                    elif self.x_offset > 0.02:
                        # self.get_logger().info(f"turning right. offset : {self.x_offset}")
                        self.twist.linear.x = 0.05 # Stop forward movement
                        self.twist.angular.z = -0.05  # Turn right
                    elif self.x_offset < -0.02:
                        # self.get_logger().info(f"turning left. offset : {self.x_offset}")
                        self.twist.linear.x = 0.05  # Stop forward movement
                        self.twist.angular.z = 0.05  # Turn left
                else:

                    if self.x_offset > 0.1:
                        # self.get_logger().info(f"turning right. offset : {self.x_offset}")

                        self.twist.linear.x = 0.05  # Stop forward movement
                        self.twist.angular.z = -0.07  # Turn right
                    elif self.x_offset < -0.1:
                        # self.get_logger().info(f"turning left. offset : {self.x_offset}")
                        self.twist.linear.x = 0.05  # Stop forward movement
                        self.twist.angular.z = 0.07  # Turn left

                self.cmd_pub.publish(self.twist)
        
        elif self.direction == 'backward':
            if self.distance >= 0.27:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
                self.send_response()
                self.aruco_toggle = False
            else:
                self.get_logger().info("Marker directly in front, moving backward.")
                self.twist.linear.x = -0.1
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
    

    def aruco_callback(self):
        if self.aruco_toggle or self.cam_toggle:
            ret, frame = self.cap.read()

            frame = cv2.resize(frame, (self.width, self.height))
            ret, buffer = cv2.imencode('.jpg', frame)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            parameters = aruco.DetectorParameters()
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=parameters)
            
            if ids is not None:
                # self.get_logger().info(f'Detected {len(ids)} markers.')
                for i in range(len(ids)):
                    marker_id = ids[i][0]

                    marker_name = self.marker_id_map.get(marker_id, "Unknown")
                    # self.get_logger().info(f'Marker detected: {marker_name}')


                    self.rvec, self.tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.025, self.matrix_coefficients, self.distortion_coefficients)
                    roll, pitch, yaw = self.get_orientation_from_rvec()
                    frame = cv2.drawFrameAxes(frame, self.matrix_coefficients, self.distortion_coefficients, self.rvec, self.tvec, 0.1)

                    
                    if self.direction == "forward" and self.target_distance == 0.0:
                        self.get_target_distance(pitch)

                    elif not self.is_mid and self.target_distance != 0.0:
                        if self.target_distance > 0:
                            rad = -1.27
                            self.get_logger().info('turn right')
                        else:
                            rad = 1.27
                            self.get_logger().info('turn left')
                        self.get_logger().info('goto target')

                        self.nav.spin(spin_dist=rad, time_allowance = 3)
                        time.sleep(2.0)
                        self.goal_msg.target.x = abs(self.target_distance * self.tvec[0][0][2] * 10)
                        self.goal_msg.time_allowance.sec = 5
                        self.goal_msg.speed = 0.05
                        goal_handle = self.drive_on_heading_client.send_goal_async(self.goal_msg)
                        time.sleep(5.0)
                        self.nav.spin(spin_dist=-rad, time_allowance = 3)
                        time.sleep(2.0)
                        self.is_mid = True

                    elif (self.location == marker_name and self.is_mid):
                        self.motor_control()
                        
                    elif self.direction == "backward":
                        self.motor_control()
            else:
                # self.get_logger().info('No markers detected.')
                pass

            # self.cam_toggle
            if self.cam_toggle:
                # Convert to ROS message
                self.img_msg.header.stamp = self.get_clock().now().to_msg()
                self.img_msg.format = "jpeg"
                self.img_msg.data = buffer.tobytes()
                self.img_pub.publish(self.img_msg)
    

    
    def get_target_distance(self, pitch):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.get_logger().info(f'find target_error_distance')
        if pitch > 0.03:
            self.twist.angular.z = -0.05  # Turn left
        elif pitch < -0.03:
            self.twist.angular.z = 0.05  # Turn right
        else:
            self.target_distance = self.tvec[0][0][0]
            self.get_logger().info(f'target_error_distance: {self.target_distance}')
        # self.get_logger().info(f'target_distance : {self.tvec[0][0][0]}')
        self.cmd_pub.publish(self.twist)


    def get_orientation_from_rvec(self):
        rotation_matrix, _ = cv2.Rodrigues(self.rvec)

        # Extract angles using rotation matrix
        roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        pitch = np.arctan2(-rotation_matrix[2, 0], np.sqrt(rotation_matrix[2, 1] ** 2 + rotation_matrix[2, 2] ** 2))
        yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

        return roll, pitch, yaw

    def get_target_distance(self, pitch):
        self.twist.linear.x = 0.0
        if pitch > 0.03:
            self.twist.angular.z = -0.02  # Turn left
        elif pitch < -0.03:
            self.twist.angular.z = 0.02  # Turn right
        else:
            self.target_distance = self.tvec[0][0][2]

        self.cmd_pub.publish(self.twist)

    def get_orientation_from_rvec(self):
        rotation_matrix, _ = cv2.Rodrigues(self.rvec)

        # Extract angles using rotation matrix
        roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        pitch = np.arctan2(-rotation_matrix[2, 0], np.sqrt(rotation_matrix[2, 1] ** 2 + rotation_matrix[2, 2] ** 2))
        yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

        return roll, pitch, yaw

def main():
    rclpy.init()
    node = ArucoCam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()