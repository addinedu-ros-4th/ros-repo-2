import time
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
import math
from std_srvs.srv import SetBool

from ament_index_python.packages import get_package_share_directory

class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')
        # ROS2 publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.img_sub = self.create_subscription(CompressedImage, '/camera/compressed', self.following_callback, 10)
        self.following_server = self.create_service(SetBool, '/following_control', self.handle_following_control)

        # Set the paths relative to the current directory
        self.model_proto_path = os.path.join(get_package_share_directory('human_following'), 'config', 'MobileNetSSD_deploy.prototxt')
        self.model_weights_path = os.path.join(get_package_share_directory('human_following'), 'config', 'MobileNetSSD_deploy.caffemodel')

        # Set parameterss
        self.twist = Twist()
        self.following_toggle = False
        self.conf_threshold = 0.5
        self.net = cv2.dnn.readNetFromCaffe(self.model_proto_path, self.model_weights_path)
        self.classNames = {15: 'person'}


    def handle_following_control(self, request, response):
        self.following_toggle = request.data
        response.success = True
        response.message = 'On' if request.data else 'Off'
        return response
        

    def following_callback(self, data):
        if self.following_toggle:
            # Decode compressed image
            np_arr = np.frombuffer(data.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            (h, w) = image.shape[:2]
            center_x = w // 2
            # Create a blob from the image
            blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)
            self.net.setInput(blob)
            detections = self.net.forward()
            person_detected = False

            for i in np.arange(0, detections.shape[2]):
                confidence = detections[0, 0, i, 2]
                
                if confidence > self.conf_threshold:
                    idx = int(detections[0, 0, i, 1])  
                    
                    if idx in self.classNames:
                        person_detected = True
                        
                        box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                        (startX, startY, endX, endY) = box.astype("int")
                        
                        # Set box points 
                        box_margin = 10
                        startX = max(0, startX - box_margin)
                        startY = max(0, startY - box_margin)
                        endX = min(w, endX + box_margin)
                        endY = min(h, endY + box_margin)

                        # Calculate Box Size
                        box_size = math.sqrt(((endX - startX)**2) + ((endY - startY)**2))
                        self.get_logger().info(f"box_size: {box_size}")

                        # Show CV2 Image
                        label = f"{self.classNames[idx]}: {confidence:.2f}"
                        cv2.rectangle(image, (startX, startY), (endX, endY), (255, 0, 0), 2)
                        y = startY - 15 if startY - 15 > 15 else startY + 15
                        cv2.putText(image, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        
                        # Calculate the deviation from the center
                        person_center_x = (startX + endX) // 2
                        deviation = person_center_x - center_x
                        self.get_logger().info(f"deviation: {deviation}")

                        # Determine action based on deviation
                        if box_size > 230:
                            self.twist.linear.x = 0.0  
                            self.twist.angular.z = 0.0  
                        
                        else:
                            if deviation < -80:
                                # Move left
                                self.twist.linear.x = 0.05  # Maintain forward velocity
                                self.twist.angular.z = 0.3  # Turn left with a higher angular velocity
                            
                            elif deviation > 80:
                                # Move right
                                self.twist.linear.x = 0.05  # Maintain forward velocity
                                self.twist.angular.z = -0.3  # Turn right with a higher angular velocity
                            
                            else:
                                if deviation < -40:
                                    # Move left
                                    self.twist.linear.x = 0.05  # Maintain forward velocity
                                    self.twist.angular.z = 0.15  # Turn left with a higher angular velocity
                            
                                elif deviation > 40:
                                    # Move right
                                    self.twist.linear.x = 0.05  # Maintain forward velocity
                                    self.twist.angular.z = -0.15  # Turn right with a higher angular velocity
                                else:
                                    # Move forward
                                    self.twist.linear.x = 0.05  # Maintain forward velocity
                                    self.twist.angular.z = 0.0  # No angular velocity

                        self.get_logger().info(f"Person detected: {confidence:.2f}")
                        self.cmd_vel_pub.publish(self.twist)

                    if not person_detected:
                        # No person detected, stop the robot
                        self.get_logger().info("No person detected")
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = 0.0
                        self.cmd_vel_pub.publish(self.twist)

        else:
            pass  

def main(args=None):
    rclpy.init()
    node = PersonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
