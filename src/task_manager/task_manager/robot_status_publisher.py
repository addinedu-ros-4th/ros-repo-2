import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import time

class RobotStatusPublisher(Node):
    def __init__(self, robot_id):
        super().__init__('robot_status_publisher_' + robot_id)
        self.robot_id = robot_id
        self.publisher_ = self.create_publisher(String, '/robot_status', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)  # 2초마다 상태를 발행
        self.status = 'available'

    def timer_callback(self):
        msg = String()
        
        self.status = random.choice(['available', 'busy'])
        msg.data = f'{self.robot_id},{self.status}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published status: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    robot_id = 'robot1'  # 로봇 ID를 설정
    robot_status_publisher = RobotStatusPublisher(robot_id)

    rclpy.spin(robot_status_publisher)

    # 종료 시 자원 해제
    robot_status_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
