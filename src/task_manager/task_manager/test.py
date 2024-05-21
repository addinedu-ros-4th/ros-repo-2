import rclpy
from rclpy.node import Node
from task_msgs.msg import RobotState  # 올바른 메시지 타입을 가져옵니다

class RobotStatePublisher(Node):

    def __init__(self):
        super().__init__('robot_state_publisher')
        self.publisher_ = self.create_publisher(RobotState, '/robot_state', 10)
        timer_period = 2.0  # 초 단위로 메시지를 발행
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = RobotState()
        msg.is_ready = True
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.is_ready}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
