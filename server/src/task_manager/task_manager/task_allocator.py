import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from task_msgs.srv import AllocateTask
from task_msgs.msg import TaskList, RobotStatus, TaskCompletion, PendingTaskList
from task_manager.task_factory import TaskFactory
from data_manager.robot_controller import RobotController
import heapq


class PriorityTask:
    def __init__(self, priority, task):
        self.priority = priority
        self.task = task

    def __lt__(self, other):
        return self.priority < other.priority


class TaskAllocator(Node):
    def __init__(self, robot_controller):
        super().__init__('task_allocator')
        
        # Subscriber
        self.task_list_sub    = self.create_subscription(TaskList,       '/task_list',       self.receive_task_list,       10)
        self.robot_status_sub = self.create_subscription(RobotStatus,    '/robot_status',    self.update_robot_status,     10)
        self.completion_sub   = self.create_subscription(TaskCompletion, '/task_completion', self.process_task_completion, 10)

        # Publisher
        self.pending_tasks_pub = self.create_publisher(PendingTaskList, '/pending_tasks', 10)
        
        # database
        self.robot_controller = robot_controller

        self.tasks = []                             # Priority queue for tasks
        self.tasks_in_progress = {}                 # Dictionary to keep track of transactions in progress
        self.robot_status = {}                      # Dictionary to keep track of robot statuses
        self.outbound_to_task_map = {}              # Dictionary to map outbound tasks to tasks
        self.inbound_to_task_map = {}               # Dictionary to map inbound tasks to unloading and storage locations
        self.pending_tasks = []                     # List of tasks awaiting assignment

    def get_final_location_from_db(self, task):
        self.robot_controller.ensure_connection()   # Ensure the connection is valid
        bundle_id = task.bundle_id
        query = "SELECT item_tag FROM Inbound WHERE inbound_id = %s"
        result = self.robot_controller.fetchone(query, (bundle_id,))
        
        if result:
            return result[0]
        else:
            self.get_logger().error("No item_tag found for the given inbound_id")
            return None


    def receive_task_list(self, msg):
        for task in msg.tasks:
            if task.task_type == 'OB':
                if task.bundle_id not in self.outbound_to_task_map:
                    self.outbound_to_task_map[task.bundle_id] = []
                self.outbound_to_task_map[task.bundle_id].append(task)

            elif task.task_type == 'IB':
                if task.bundle_id not in self.inbound_to_task_map:
                    self.inbound_to_task_map[task.bundle_id] = []
                self.inbound_to_task_map[task.bundle_id].append(task)
                self.inbound_item = task.item

        self.get_logger().info(f'Received task list with {len(msg.tasks)} tasks')
        self.bundle_tasks()             # Bundling
        self.update_pending_tasks()     # Update pending list
        self.allocate_transaction()     # Allocate task to Robot


    # Tasks grouped by task type (Inbound, Outbound)
    def bundle_tasks(self):
        # In case of outbound
        for bundle_id, tasks in self.outbound_to_task_map.items():
            transaction_tasks = TaskFactory.create_outbound_tasks(bundle_id, tasks)              
            
            if transaction_tasks:                                                   # Create transaction task
                unique_tasks = list({task.task_id: task for task in transaction_tasks}.values())
                priority = min(task.priority for task in unique_tasks)
                heapq.heappush(self.tasks, PriorityTask(priority, unique_tasks))
        self.outbound_to_task_map.clear()

        # In case of inbound
        for bundle_id, tasks in self.inbound_to_task_map.items():
            transaction_tasks = TaskFactory.create_inbound_tasks(bundle_id, self.inbound_item, tasks, self.get_final_location_from_db(tasks[0]))
            
            if transaction_tasks:                                                   # Create transaction task
                unique_tasks = list({task.task_id: task for task in transaction_tasks}.values())
                priority = min(task.priority for task in unique_tasks)
                heapq.heappush(self.tasks, PriorityTask(priority, unique_tasks))
        self.inbound_to_task_map.clear()


    def update_robot_status(self, msg):
        self.robot_status[msg.robot_id] = msg.robot_status
        self.robot_controller.update_robot_status(msg.robot_id, msg.robot_status)   # Update Robot Status Database
        self.get_logger().info(f'Received status from {msg.robot_id}: {msg.robot_status}')
        
        self.allocate_transaction()                                                       # Allocate task


    # Assign work by determining robot status
    def allocate_transaction(self):
        available_robots = [robot_id for robot_id, status in self.robot_status.items() if status == 'available']

        while available_robots and self.tasks:
            priority_task = heapq.heappop(self.tasks)   # Get the highest priority task
            tasks = priority_task.task                  # Access the list of tasks
            robot_id = available_robots.pop(0)          # Get the first available robot

            if self.robot_status[robot_id] == "busy":
                available_robots.insert(0, robot_id)
                continue

            # Immediately change status to prevent reallocation
            self.robot_status[robot_id] = "busy"

            self.assign_transaction(tasks, robot_id)
            
        self.update_pending_tasks()                     # Update pending list


    def assign_transaction(self, tasks, robot_id):
        transaction_id = f'{tasks[0].task_id.split("_")[0]}_{robot_id}'
        self.tasks_in_progress[transaction_id] = (tasks, robot_id)
        self.tasks_assigned = {task.task_id: False for task in tasks}
        self.get_logger().info(f'Assigned transaction {transaction_id} with {len(tasks)} tasks to robot {robot_id}')
        
        first_task = tasks[0]

        self.assign_task(first_task, robot_id, transaction_id)


    # One robot is responsible for one transaction
    def assign_task(self, task, robot_id, transaction_id=None):
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
        request.lift = task.lift

        service_name = f'/allocate_task_{robot_id}'                                 # Allocate task service
        allocate_task_client = self.create_client(AllocateTask, service_name)       # Create service client

        future = allocate_task_client.call_async(request)
        future.add_done_callback(lambda future: self.process_task_allocation_response(future, task, robot_id, transaction_id))

        # Update robot status database
        self.robot_controller.update_robot_status(robot_id, 'busy')


    def process_task_allocation_response(self, future, task, robot_id, transaction_id):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Task allocation successful for task {task.task_id}')
                self.tasks_assigned[task.task_id] = True                                # Success Allocation status save
                self.get_logger().info(f'Waiting for task {task.task_id} completion by robot {robot_id}')
            else:
                self.get_logger().warn(f'Task allocation failed for task {task.task_id}')
                self.requeue_task(task)
                self.robot_status[robot_id] = "available"
        except Exception as e:
            self.get_logger().error(f'Task allocation service call failed for task {task.task_id}: {e}')
            self.requeue_task(task)
            self.robot_status[robot_id] = "available"
        finally:
            self.update_pending_tasks()
            

    def process_task_completion(self, msg):
        self.get_logger().info(f'Received task completion for task {msg.task_id} by robot {msg.robot_id}')

        transaction_id = f'{msg.task_id.split("_")[0]}_{msg.robot_id}'

        if transaction_id in self.tasks_in_progress:
            tasks, _ = self.tasks_in_progress[transaction_id]

            if not msg.success:
                self.get_logger().warn(f'Task {msg.task_id} failed, reassigning transaction {transaction_id}')
                self.reassign_transaction(transaction_id)
            else:
                self.tasks_assigned[msg.task_id] = True

                next_task_index = next((i for i, t in enumerate(tasks) if not self.tasks_assigned[t.task_id]), None)

                if next_task_index is not None:
                    next_task = tasks[next_task_index]
                    self.get_logger().info(f'Assigning next task {next_task.task_id} to robot {msg.robot_id} for transaction {transaction_id}')
                    self.assign_task(next_task, msg.robot_id, transaction_id)
                else:
                    self.get_logger().info(f'All tasks in transaction {transaction_id} completed')
                    del self.tasks_in_progress[transaction_id]
                    self.robot_status[msg.robot_id] = "available"
                    # Update robot status database
                    self.robot_controller.update_robot_status(msg.robot_id, 'busy')

                    self.allocate_transaction()
        else:
            self.get_logger().warn(f'Transaction {transaction_id} not found in progress')
            
            
    def requeue_task(self, task):
        heapq.heappush(self.tasks, PriorityTask(task.priority, [task]))
        self.update_pending_tasks()


    def reassign_transaction(self, transaction_id):
        if transaction_id in self.tasks_in_progress:
            tasks, robot_id = self.tasks_in_progress[transaction_id]

            for task in tasks:
                self.requeue_task(task)

            del self.tasks_in_progress[transaction_id]
            self.robot_status[robot_id] = 'available'
            self.get_logger().info(f'Reassigning tasks for transaction {transaction_id} due to failure')
            self.allocate_transaction()

    
    def update_pending_tasks(self):
        self.pending_tasks = [priority_task.task for priority_task in self.tasks]
        self.publish_pending_tasks()
        
    
    def publish_pending_tasks(self):
        msg = PendingTaskList()
        for tasks in self.pending_tasks:
            msg.tasks.extend(tasks)
        self.pending_tasks_pub.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController(host='localhost')
    task_allocator = TaskAllocator(robot_controller)

    # MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(task_allocator)
    executor.spin()

    task_allocator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
