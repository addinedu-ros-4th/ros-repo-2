import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from task_msgs.msg import TaskList, RobotStatus, TaskCompletion
from task_msgs.srv import AllocateTask
from task_manager.task_factory import TaskFactory
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
        self.completion_sub = self.create_subscription(TaskCompletion, '/task_completion', self.transaction_completion_callback, 10)

        self.tasks = []               # Priority queue for tasks
        self.tasks_in_progress = {}   # Dictionary to keep track of tasks in progress
        self.robot_status = {}        # Dictionary to keep track of robot statuses
        self.outbound_to_task_map = {}   # Dictionary to map orders to tasks
        self.inbound_to_task_map = {} # Dictionary to map inbound tasks to unloading and storage locations


    def get_final_location_from_db(self, task):
        return 'A1'
    
    
    # Receive and categorize tasks by type
    def task_list_callback(self, msg):
        for task in msg.tasks:
            if task.task_type == 'OB':
                if task.bundle_id not in self.outbound_to_task_map:
                    self.outbound_to_task_map[task.bundle_id] = []
                self.outbound_to_task_map[task.bundle_id].extend(TaskFactory.create_outbound_tasks(task))
                
            elif task.task_type == 'IB':
                if task.bundle_id not in self.inbound_to_task_map:
                    self.inbound_to_task_map[task.bundle_id] = []
                initial_location = self.get_final_location_from_db(task)
                self.inbound_to_task_map[task.bundle_id].extend(TaskFactory.create_inbound_tasks(task, initial_location))

            else:
                self.get_logger().info(f'Unknown Task {task.task_type}')
                
        self.get_logger().info(f'Received task list with {len(msg.tasks)} tasks')
        self.bundle_tasks()
        self.allocate_tasks()


    # Bundle outbound tasks by user ID and add to priority queue
    def bundle_tasks(self):
        for bundle_id, tasks in self.outbound_to_task_map.items():
            priority = min(task.priority for task in tasks)
            for task in tasks:
                heapq.heappush(self.tasks, PriorityTask(priority, tasks))
        self.outbound_to_task_map.clear()

        # Bundle inbound tasks by bundle ID
        for tasks in self.inbound_to_task_map.values():
            priority = min(task.priority for task in tasks)
            for task in tasks:
                heapq.heappush(self.tasks, PriorityTask(task.priority, tasks))
        self.inbound_to_task_map.clear()
    
    
    # Update Robot status
    def robot_status_callback(self, msg):
        self.robot_status[msg.robot_id] = msg.robot_status
        self.get_logger().info(f'Received status from {msg.robot_id}: {msg.robot_status}')
        self.allocate_tasks()
            
            
    # Judgement Robot status and task list, allocate tasks
    def allocate_tasks(self):
        available_robots = [robot_id for robot_id, status in self.robot_status.items() if status == 'available'] 
        
        while available_robots and self.tasks:
            priority_task = heapq.heappop(self.tasks)        # Get the highest priority task
            tasks = priority_task.task                       # Access the list of tasks 
            task_type = tasks[0].task_type
            robot_id = available_robots.pop(0)               # Get the first available robot
            
            if self.robot_status[robot_id] == "busy":
                available_robots.insert(0, robot_id)
                continue
            
            # Immediately change status to prevent reallocation
            self.robot_status[robot_id] = "busy"
            
            self.assign_transaction_to_robot(tasks, robot_id)
            

    # Send task allocation request to the robot
    def assign_transaction_to_robot(self, tasks, robot_id):
        transaction_id = f'{tasks[0].task_id.split("_")[0]}_{robot_id}'
        self.tasks_in_progress[transaction_id] = (tasks, robot_id)
        self.tasks_assigned = {task.task_id: False for task in tasks}
        
        first_task = tasks[0]
        
        self.assign_task_to_robot(first_task, robot_id, transaction_id)
        
    
    def assign_task_to_robot(self, task, robot_id, transaction_id=None):
        # Send task allocation request to the robot
        if transaction_id is None:
            transaction_id = f'{task.task_id.split("_")[0]}_{robot_id}'
        if transaction_id not in self.tasks_in_progress:
            self.tasks_in_progress[transaction_id] = ([task], robot_id)
        else:
            self.tasks_in_progress[transaction_id][0].append(task)
        
        request = AllocateTask.Request()
        request.task_id = task.task_id
        request.task_type = task.task_type
        request.priority = task.priority
        request.item = task.item
        request.quantity = task.quantity
        request.location = task.location
        
        self.get_logger().info(f'Assigning task {task.task_id} to robot {robot_id}')
        service_name = f'/allocate_task_{robot_id}'
        allocate_task_client = self.create_client(AllocateTask, service_name)
        
        future = allocate_task_client.call_async(request)
        future.add_done_callback(lambda future, t=task: self.task_allocation_response(future, t, robot_id, transaction_id))
    
    
    # Each task process
    def task_allocation_response(self, future, task, robot_id, transaction_id):
        # Handle the response of the task allocation
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Task allocation successful for task {task.task_id}')
                self.tasks_assigned[task.task_id] = True    # Each task's success status update

                next_task_index = next((i for i, t in enumerate(self.tasks_in_progress[transaction_id][0]) if not self.tasks_assigned[t.task_id]), None)
                if next_task_index is not None:
                    next_task = self.tasks_in_progress[transaction_id][0][next_task_index]
                    self.assign_task_to_robot(next_task, robot_id, transaction_id)
                else:
                    del self.tasks_in_progress[transaction_id]
                    self.robot_status[robot_id] = "available"
                    self.get_logger().info(f'Transaction {transaction_id} completed successfully')
                    self.allocate_tasks()
            
            else:
                self.get_logger().warn(f'Task allocation failed for task {task.task_id}: {response.message}')
                self.requeue_task(task)
                self.robot_status[robot_id] = "available"
                
        except Exception as e:
            self.get_logger().error(f'Task allocation service call failed for task {task.task_id}: {e}')
            self.requeue_task(task)
            # Failed task allocation
            self.robot_status[robot_id] = "available"
            

    # Determine success or failure transaction
    def transaction_completion_callback(self, msg):
        transaction_id = f'{msg.task_id}_{msg.robot_id}'
        
        if transaction_id in self.tasks_in_progress:
            tasks, robot_id = self.tasks_in_progress[transaction_id]
            
            if not msg.success:
                self.reassign_task(transaction_id)
            else:
                self.tasks_assigned[msg.task_id] = False
                
            # Next task start
            next_task_index = next((i for i, t in enumerate(tasks) if not self.tasks_assigned[t.task_id]), None)
            
            if next_task_index is not None:
                next_task = tasks[next_task_index]
                self.assign_task_to_robot(next_task, robot_id, transaction_id)
            else:
                del self.tasks_in_progress[transaction_id]
                self.robot_status[robot_id] = "available"
                self.get_logger().info(f'Transaction {transaction_id} completed successfully')
                self.allocate_tasks()
    
    
    # Requeue a task that failed to be allocated     
    def requeue_task(self, task):
        heapq.heappush(self.tasks, PriorityTask(task.priority, [task]))
        
    
    # Reassign tasks in case of failure
    def reassign_task(self, transaction_id):
        if transaction_id in self.tasks_in_progress:
            tasks, robot_id = self.tasks_in_progress[transaction_id]
            
            for task in tasks:
                self.requeue_task(task)
                
            del self.tasks_in_progress[transaction_id]
            self.robot_status[robot_id] = 'available'       # Update robot status
            self.get_logger().info(f'Reassigning tasks for transaction {transaction_id} due to failure')
            self.allocate_tasks()    


def main(args=None):
    rclpy.init(args=args)
    task_allocator = TaskAllocator()
    rclpy.spin(task_allocator)
    task_allocator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
