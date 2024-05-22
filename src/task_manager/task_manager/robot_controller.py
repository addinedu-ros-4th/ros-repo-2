import rclpy
from rclpy.node import Node
from task_msgs.msg import RobotState

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        self.robot_ready = False

        self.state_subscription = self.create_subscription(
            RobotState,
            '/task_success',
            self.state_callback,
            10
        )

    def state_callback(self, msg):
        self.robot_ready = msg.is_ready
        # 로봇 상태 변경 후 db 변경

    def is_robot_ready(self):
        return self.robot_ready


def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()