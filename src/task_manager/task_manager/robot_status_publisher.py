import rclpy
from rclpy.node import Node
from task_msgs.msg import RobotStatus
import random

class RobotStatusPublisher(Node):
    def __init__(self, robot_id):
        super().__init__('robot_status_publisher_' + robot_id)
        self.robot_id = robot_id
        self.publisher_ = self.create_publisher(RobotStatus, '/robot_status', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)  # Published status every 2 seconds
        self.robot_status = 'available'

    def timer_callback(self):
        msg = RobotStatus()
        
        self.robot_status = random.choice(['available', 'busy', 'charge'])
        msg.robot_id = self.robot_id
        msg.robot_status = self.robot_status
        self.publisher_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    robot_id = '213'  # Robot ID = Domain ID
    # robot_id = '214'
    # robot_id = '215'
    robot_status_publisher = RobotStatusPublisher(robot_id)

    rclpy.spin(robot_status_publisher)

    robot_status_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
