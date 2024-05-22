import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from task_msgs.msg import TaskList, RobotStatus
from task_msgs.srv import AllocateTask
import heapq

class TaskAllocator(Node):
    def __init__(self):
        super().__init__('task_allocator')
        self.subscription = self.create_subscription(TaskList, '/task_list', self.task_list_callback, 10)
        self.robot_status_sub = self.create_subscription(RobotStatus, '/robot_status', self.robot_status_callback, 10)
        self.allocate_task_client = self.create_client(AllocateTask, '/allocate_task')

        self.tasks = []              # Priority queue for tasks
        self.tasks_in_progress = {}  # Dictionary to keep track of tasks in progress
        self.robot_status = {}       # Dictionary to keep track of robot statuses


    # Management task list heap based on priority
    def task_list_callback(self, msg):
        for task in msg.tasks:
            heapq.heappush(self.tasks, (task.priority, task))  # Add task to priority queue
        self.get_logger().info(f'Received task list with {len(msg.tasks)} tasks')
        self.allocate_tasks()


    # Management Robot status
    def robot_status_callback(self, msg):
        robot_id, is_ready = msg.data.split(',')
        self.robot_status[robot_id] = is_ready
        self.allocate_tasks()
    
    
    def allocate_tasks(self):
        # if task list도 있고 robot 리스트에 true인 거 있으면:
        #     allocate task에 request해
        #     뭐라고? robot list에 있는 robot id 그리고 task list에 있는 것들~
            
        # else
            return


def main(args=None):
    rclpy.init(args=args)
    task_allocator = TaskAllocator()
    rclpy.spin(task_allocator)
    task_allocator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
