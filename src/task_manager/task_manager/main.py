import rclpy
from rclpy.executors import MultiThreadedExecutor
from task_manager.robot_controller import RobotController
from task_manager.task_dispatcher import TaskDispatcher

def main(args=None):
    rclpy.init(args=args)

    robot_controller = RobotController()
    task_dispatcher = TaskDispatcher()

    executor = MultiThreadedExecutor()
    executor.add_node(robot_controller)
    executor.add_node(task_dispatcher)

    robot_controller.get_logger().info('Starting RobotController and TaskDispatcher nodes.')

    try:
        executor.spin()
    finally:
        executor.shutdown()
        robot_controller.destroy_node()
        task_dispatcher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
