import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class RobotCam(Node):
    def __init__(self):
        super().__init__('img_publisher')
        
        self.publisher = self.create_publisher(CompressedImage, '/camera/compressed', 10) 
        self.timer = self.create_timer(0.01, self.time_callback)

        # videocapture instance
        self.fps = 25
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        # cv_bridge, img_msg
        self.cv_bridge = CvBridge()
        self.img_msg = CompressedImage()

        # Set parameters
        self.declare_parameter('width', 320)
        self.width = self.get_parameter('width').value
        self.declare_parameter('length', 240)
        self.length = self.get_parameter('length').value

        # log info
        output_msg = f"Video Width : {self.width}\n\rVideo Length : {self.length}"
        self.get_logger().info(output_msg)
    

    def time_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return
        frame = cv2.resize(frame, (self.width, self.length))
        # Compress the frame
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            self.get_logger().error("Failed to compress image")
            return
        # Convert to ROS message
   
        self.img_msg.header.stamp = self.get_clock().now().to_msg()
        self.img_msg.format = "jpeg"
        self.img_msg.data = buffer.tobytes()
        self.publisher.publish(self.img_msg)


def main():
    rclpy.init()
    node = RobotCam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()