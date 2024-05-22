import rclpy
from rclpy.node import Node
from task_msgs_msgs.srv import TaskSuccess
from task_msgs_msgs.msg import TaskList

class TaskSuccessHandler(Node):
    def __init__(self):
        super().__init__('task_success_handler')
        self.task_success_client = self.create_client(TaskSuccess, '/task_success')
        self.task_list_publisher = self.create_publisher(TaskList, '/task_list', 10)
        self.task_list = []

    def check_task_success(self, task_id):
        request = TaskSuccess.Request()
        request.task_id = task_id
        future = self.task_success_client.call_async(request)
        future.add_done_callback(self.task_success_response_callback)

    def task_success_response_callback(self, future):
        response = future.result()
        if response.success:
            self.remove_task(response.task_id)

    def remove_task(self, task_id):
        self.task_list = [task for task in self.task_list if task.task_id != task_id]
        task_list_msg = TaskList(tasks=self.task_list)
        self.task_list_publisher.publish(task_list_msg)

def main():
    rclpy.init()
    node = TaskSuccessHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
