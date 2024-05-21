import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Robot1(Node):
    def __init__(self):
        super().__init__('robot1')
        self.publisher_ = self.create_publisher(String, '/pose', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0.0

    def timer_callback(self):
        msg = String()
        msg.data = f"Robot1 position: {self.i}"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1.0

def main(args=None):
    rclpy.init(args=args)
    node = Robot1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
