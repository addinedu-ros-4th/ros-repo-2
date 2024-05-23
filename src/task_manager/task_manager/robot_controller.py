import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from my_interfaces.srv import AllocateTask, TaskCompletion

class RobotNode(Node):
    def __init__(self, robot_id):
        super().__init__('robot_node_' + robot_id)
        self.robot_id = robot_id
        self.status_publisher = self.create_publisher(String, '/robot_status', 10)
        self.completion_publisher = self.create_publisher(TaskCompletion, '/task_completion', 10)
        self.create_service(AllocateTask, '/allocate_task', self.handle_allocate_task)
        self.create_timer(2.0, self.publish_status)
        self.current_task = None

    def publish_status(self):
        status_msg = String()
        if self.current_task is None:
            status_msg.data = f'{self.robot_id},available'
        else:
            status_msg.data = f'{self.robot_id},busy'
        self.status_publisher.publish(status_msg)

    def handle_allocate_task(self, request, response):
        self.current_task = request
        self.get_logger().info(f'Received task {request.task_id}')
        self.perform_task()
        response.success = True
        response.message = 'Task received'
        return response

    def perform_task(self):
        # 실제 작업 수행 로직을 여기에 추가합니다.
        self.get_logger().info(f'Performing task {self.current_task.task_id}')
        # 작업 수행 시뮬레이션을 위해 잠시 대기
        import time
        try:
            time.sleep(5)  # 작업 수행 시간
            self.get_logger().info(f'Task {self.current_task.task_id} completed')
            self.report_task_completion(success=True, message='Task completed successfully')
        except Exception as e:
            self.get_logger().error(f'Error performing task {self.current_task.task_id}: {e}')
            self.report_task_completion(success=False, message='Task failed')

    def report_task_completion(self, success, message):
        completion_msg = TaskCompletion()
        completion_msg.task_id = self.current_task.task_id
        completion_msg.robot_id = self.robot_id
        completion_msg.success = success
        completion_msg.message = message
        self.completion_publisher.publish(completion_msg)
        self.current_task = None

def main(args=None):
    rclpy.init(args=args)
    robot_id = 'robot1'  # 로봇 ID를 고유하게 설정
    robot_node = RobotNode(robot_id)
    rclpy.spin(robot_node)
    robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
