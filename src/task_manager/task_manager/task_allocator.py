import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from task_msgs.msg import TaskList, RobotStatus, TaskCompletion
from task_msgs.srv import AllocateTask
import heapq

class PriorityTask:
    def __init__(self, priority, task):
        self.priority = priority
        self.task = task

    def __lt__(self, other):
        return self.priority < other.priority
    
class TaskAllocator(Node):
    def __init__(self):
        super().__init__('task_allocator')
        self.task_list_sub = self.create_subscription(TaskList, '/task_list', self.task_list_callback, 10)
        self.robot_status_sub = self.create_subscription(RobotStatus, '/robot_status', self.robot_status_callback, 10)
        self.completion_sub = self.create_subscription(TaskCompletion, '/task_completion', self.task_completion_callback, 10)
        self.allocate_task_client = self.create_client(AllocateTask, '/allocate_task')

        self.tasks = []              # Priority queue for tasks
        self.tasks_in_progress = {}  # Dictionary to keep track of tasks in progress
        self.robot_status = {}       # Dictionary to keep track of robot statuses


    # Management task list heap based on priority
    def task_list_callback(self, msg):
        for task in msg.tasks:
            heapq.heappush(self.tasks, PriorityTask(task.priority, task))  # Add task to priority queue
        self.get_logger().info(f'Received task list with {len(msg.tasks)} tasks')
        self.allocate_tasks()


    # Management Robot status
    def robot_status_callback(self, msg):
        robot_id = msg.robot_id
        robot_status = msg.robot_status
        self.robot_status[robot_id] = robot_status
        self.allocate_tasks()
            
            
    # Judgement Robot status and task list, allocate tasks
    def allocate_tasks(self):
        available_robots = [robot_id for robot_id, status in self.robot_status.items() if status == 'available']
        if not available_robots and not self.tasks:
            return    
        
        while available_robots and self.tasks:
            priority_task = heapq.heappop(self.tasks)        # Get the highest priority task
            task = priority_task.task
            robot_id = available_robots.pop(0)               # Get the first available robot
            self.assign_task_to_robot(task, robot_id)


    # Task Assign request 
    def assign_task_to_robot(self, task, robot_id):
        request = AllocateTask.Request()
        request.task_id = task.task_id
        request.task_type = task.task_type
        request.priority = task.priority
        request.item = task.item
        request.quantity = task.quantity
        request.location = task.location
        
        self.get_logger().info(f'Assigning task {task.task_id} to robot {robot_id}')
        future = self.allocate_task_client.call_async(request) # asyncronize
        future.add_done_callback(lambda future: self.task_allocation_response(future, task, robot_id))
    
    
    def task_allocation_response(self, future, task, robot_id):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Task allocation successful')
                self.tasks_in_progress[task.task_id] = (task, robot_id)  # Add task to in-progress list
                self.robot_status[robot_id] = "buzy"                     # Update robot status
            else:
                self.get_logger().warn(f'Task allocation failed: {response.message}')
                heapq.heappush(self.tasks, (task.priority, task))        # Re-add task to queue
        except Exception as e:
            self.get_logger().error(f'Task allocation service call failed: {e}')
            heapq.heappush(self.tasks, (task.priority, task))   
            
            
    def task_completion_callback(self, msg):
        task_id = msg.task_id
        robot_id = msg.robot_id

        if task_id in self.tasks_in_progress:
            del self.tasks_in_progress[task_id]        # Remove task from in-progress list
            self.robot_status[robot_id] = 'available'  # Update robot status
            self.get_logger().info(f'Task {task_id} completed by robot {robot_id}')
            if not msg.success:
                self.reassign_task(task_id)            # Reassign the task if it was not successful
            self.allocate_tasks()
            
     
    def reassign_task(self, task_id):
        if task_id in self.tasks_in_progress:
            task, robot_id = self.tasks_in_progress[task_id]
            heapq.heappush(self.tasks, (task.priority, task))  # Re-add task to queue
            del self.tasks_in_progress[task_id]
            self.robot_status[robot_id] = 'available'          # Update robot status
            self.get_logger().info(f'Reassigning task {task_id} due to failure')      


def main(args=None):
    rclpy.init(args=args)
    task_allocator = TaskAllocator()
    rclpy.spin(task_allocator)
    task_allocator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
