import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2

class RobotCam(Node):
    def __init__(self):
        super().__init__('robot_cam')
        self.publisher = self.create_publisher(
                                    Image, 
                                    '/camera', 
                                    10)
        time_period = 0.01
        self.timer = self.create_timer(time_period, 
                                    self.time_callback)
        
        self.cap = cv2.VideoCapture(0)
        self.cv_bridge = CvBridge()

        self.declare_parameter('width', 640)
        self.width = self.get_parameter('width').value
        self.declare_parameter('length', 480)
        self.length = self.get_parameter('length').value

        output_msg = "Video Width : " + str(self.width) + "\n\r"
        output_msg = output_msg + "Video Length : " + str(self.length)
        self.get_logger().info(output_msg)

    def time_callback(self):
        ret, frame = self.cap.read()
        frame = cv2.resize(frame, (self.width, self.length))
        img = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher.publish(img)
        

def main() :
    rclpy.init()
    node = RobotCam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()