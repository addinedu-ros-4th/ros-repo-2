#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from cv2 import aruco
from geometry_msgs.msg import Pose
import os

marker_id_map = {
    0: "I1", 1: "I2", 2: "I3",
    3: "O1", 4: "O2", 5: "O3",
    6: "R1", 7: "R2",
    8: "A1", 9: "A2", 10: "A3",
    11: "B1", 12: "B2", 13: "B3",
    14: "C1", 15: "C2", 16: "C3",
    17: "P1", 18: "P2", 19: "P3"
}

class ImageConverter(Node):

    def __init__(self):
        super().__init__('image_converter')

        self.pose_pub = self.create_publisher(Pose, 'pose', 1)
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera', self.callback, 1)
        
        # Load calibration data
        npz_path = '/home/yongtak_ras/test/picamera/calibration_data/MultiMatrix.npz'
        if not os.path.exists(npz_path):
            self.get_logger().error(f'Calibration file not found: {npz_path}')
            raise FileNotFoundError(f'Calibration file not found: {npz_path}')
        with np.load(npz_path) as data:
            self.matrix_coefficients = data['camMatrix']
            self.distortion_coefficients = data['distCoef']
        
        # Declare parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('length', 480)

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
                    self.get_logger().info(f'Marker detected: {marker_name}')
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.1, self.matrix_coefficients, self.distortion_coefficients)
                    R, _ = cv2.Rodrigues(rvec)
                    R = np.matrix(R).T
                    tvec1 = np.reshape(tvec, (3, 1))
                    t = np.hstack((R, tvec1))
                    t = np.vstack((t, [0, 0, 0, 1]))
                    tt = np.linalg.inv(t)
                    ct = np.array([tt[0, 3], tt[1, 3], tt[2, 3]])
                    aruco.drawDetectedMarkers(cv_image, corners, ids)
                    cv2.drawFrameAxes(cv_image, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec, 0.05)
                    
                    p.position.x = ct[0]
                    p.position.y = ct[1]
                    p.position.z = ct[2]
                    p.orientation.x = 0.0
                    p.orientation.y = 0.0
                    p.orientation.z = 0.0
                    p.orientation.w = 1.0

                    # Draw marker name on the image
                    cv2.putText(cv_image, marker_name, (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

                    # Draw arrows based on the marker's position
                    if tvec[0][0][0] > 0.01:
                        cv2.arrowedLine(cv_image, (490, 240), (590, 240), (138, 43, 226), 3)
                    elif tvec[0][0][0] < -0.01:
                        cv2.arrowedLine(cv_image, (150, 240), (50, 240), (138, 43, 226), 3)

                    # Check if the marker is front
                    if -0.01 < tvec[0][0][0] < 0.01 and -0.01 < tvec[0][0][1] < 0.01:
                        cv2.putText(cv_image, "Front", (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 20), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

                # Calculate distances
                left_distance = min(tvec[0][0][0], 0) if tvec[0][0][0] < 0 else 0
                right_distance = max(tvec[0][0][0], 0) if tvec[0][0][0] > 0 else 0
                front_distance = tvec[0][0][2]

                # Display distances on the image
                cv2.putText(cv_image, f'Left: {left_distance:.2f}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(cv_image, f'Right: {right_distance:.2f}', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(cv_image, f'Front: {front_distance:.2f}', (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

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
