import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from task_msgs.srv import AllocateTask
from task_msgs.msg import TaskList, RobotStatus, TaskCompletion, Task
from task_manager.task_factory import TaskFactory
from data_manager.robot_controller import RobotController
import heapq
import json
import threading


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
        self.task_list_sub    = self.create_subscription(TaskList,       '/task_list',       self.receive_task_list,      10)
        self.robot_status_sub = self.create_subscription(RobotStatus,    '/robot_status',    self.robot_status_callback,  10)
        self.completion_sub   = self.create_subscription(TaskCompletion, '/task_completion', self.handle_task_completion, 10)
        
        # Publisher
        self.current_transactions_pub = self.create_publisher(String,      '/current_transactions', 10)
        self.unassigned_tasks_pub     = self.create_publisher(TaskList,    '/unassigned_tasks',     10)
        self.robot_status_pub         = self.create_publisher(RobotStatus, '/robot_status'        , 10)
        
        # Database controller
        self.robot_controller = robot_controller

        # Data
        self.tasks = []                             # Priority queue for tasks
        self.tasks_in_progress = {}                 # Dictionary to keep track of transactions in progress
        self.robot_status = {}                      # Dictionary to keep track of robot statuses
        self.outbound_to_task_map = {}              # Dictionary to map outbound tasks to tasks
        self.inbound_to_task_map = {}               # Dictionary to map inbound tasks to unloading and storage locations

        # Timer
        self.idle_timers = {}                       # Dictionary to keep track of timers for each robot
        
        # Initialize states
        self.initialize_states()
        

    def initialize_states(self):
        # Clear in-progress tasks
        self.tasks_in_progress.clear()
        self.tasks_assigned = {}

        # Publish empty current transactions to reset the GUI
        self.publish_current_transactions()
        self.publish_unassigned_tasks()
        
        
    def reset_timer(self, robot_id):
        if robot_id in self.idle_timers:
            self.idle_timers[robot_id].cancel()
        self.idle_timers[robot_id] = threading.Timer(5.0, self.assign_charge_task, args=[robot_id])
        self.idle_timers[robot_id].start()
    
    
    def assign_charge_task(self, robot_id):
        # Check if the robot is still available
        if self.robot_status.get(robot_id) == "available":
            charge_task = Task(task_id='charge_task', task_type='CG', priority=1, bundle_id='CG', item='', quantity=0, location='P1', lift='X')
            self.assign_task(charge_task, robot_id)
        
        
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
        self.bundle_tasks()
        self.publish_unassigned_tasks()
        self.allocate_transaction()

        # Reset the timer if a new task is received
        for robot_id, status in self.robot_status.items():
            if status == "available":
                self.reset_timer(robot_id)
                
                
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
            
            # Robot status topic publish
            msg = RobotStatus()
            msg.robot_id = robot_id
            msg.robot_status = "busy"
            self.robot_status_pub.publish(msg)
            
            # Assign
            self.assign_transaction(tasks, robot_id)


    def assign_transaction(self, tasks, robot_id):
        # Create Transaction ID (Consumer, Manager)
        if tasks[0].task_type == 'IB':
            prefix = 'I'
        else:
            prefix = 'O'
            
        transaction_id = f'{prefix}{tasks[0].bundle_id}_{robot_id}'
        self.tasks_in_progress[transaction_id] = (tasks, robot_id)
        self.tasks_assigned = {task.task_id: False for task in tasks}
        self.get_logger().info(f'Assigned transaction {transaction_id} with {len(tasks)} tasks to robot {robot_id}')
        
        # Publish current transactions immediately after assignment
        self.publish_current_transactions()
        
        first_task = tasks[0]
        self.assign_task(first_task, robot_id, transaction_id)


    # One robot is responsible for one transaction
    def assign_task(self, task, robot_id, transaction_id=None):
        if transaction_id is None:
            if task.task_type == 'IB':
                prefix = 'I'
            else:
                prefix = 'O'
            transaction_id = f'{prefix}{task.bundle_id}_{robot_id}'

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


    def requeue_task(self, task):
        heapq.heappush(self.tasks, PriorityTask(task.priority, [task]))
        self.publish_unassigned_tasks()


    def reassign_transaction(self, transaction_id):
        if transaction_id in self.tasks_in_progress:
            tasks, robot_id = self.tasks_in_progress[transaction_id]

            for task in tasks:
                self.requeue_task(task)

            del self.tasks_in_progress[transaction_id]
            self.robot_status[robot_id] = 'available'
            self.get_logger().info(f'Reassigning tasks for transaction {transaction_id} due to failure')
            self.allocate_transaction()


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
        
    
    def robot_status_callback(self, msg):
        self.robot_status[msg.robot_id] = msg.robot_status
        self.robot_controller.update_robot_status(msg.robot_id, msg.robot_status)   # Update Robot Status Database
        self.get_logger().info(f'Received status from {msg.robot_id}: {msg.robot_status}')
        
        if msg.robot_status == "available":
            self.reset_timer(msg.robot_id)                                          # Reset timer for available robots
    
        self.allocate_transaction()                                                 # Allocate task
        
        
    def process_task_allocation_response(self, future, task, robot_id, transaction_id):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Task allocation successful for task {task.task_id}')
                self.tasks_assigned[task.task_id] = True                            # Success Allocation status save
                self.get_logger().info(f'Waiting for task {task.task_id} completion by robot {robot_id}')
            else:
                self.get_logger().warn(f'Task allocation failed for task {task.task_id}')
                self.requeue_task(task)
                self.robot_status[robot_id] = "available"
                # self.robot_status_sub.publish(robot_id, "available")
        except Exception as e:
            self.get_logger().error(f'Task allocation service call failed for task {task.task_id}: {e}')
            self.requeue_task(task)
            self.robot_status[robot_id] = "available"
        finally:
            self.publish_unassigned_tasks()
            
            
    def publish_current_transactions(self, transaction_id=None):
        transactions_info = []

        if transaction_id:
            if transaction_id in self.tasks_in_progress:
                tasks, robot_id = self.tasks_in_progress[transaction_id]
                tasks_info = [{"task_id": task.task_id, "location": task.location, "completed": self.tasks_assigned.get(task.task_id, False)} for task in tasks]
                transactions_info.append({"transaction_id": transaction_id, "robot_id": robot_id, "tasks": tasks_info})
        else:
            for transaction_id, (tasks, robot_id) in self.tasks_in_progress.items():
                tasks_info = [{"task_id": task.task_id, "location": task.location, "completed": self.tasks_assigned.get(task.task_id, False)} for task in tasks]
                transactions_info.append({"transaction_id": transaction_id, "robot_id": robot_id, "tasks": tasks_info})

        if transactions_info:
            msg = String(data=json.dumps(transactions_info))
            self.current_transactions_pub.publish(msg)
            self.get_logger().info(f'Published current transactions: {transactions_info}')
        else:
            self.get_logger().info(f'All transactions are completed. Published empty transactions list.')


    def publish_unassigned_tasks(self):
        unassigned_tasks = [task for priority_task in self.tasks for task in priority_task.task if isinstance(task, Task)]
        task_list_msg = TaskList(tasks=unassigned_tasks)
        self.unassigned_tasks_pub.publish(task_list_msg)


    def handle_task_completion(self, msg):
        self.process_task_completion(msg)
        self.additional_task_completion_processing(msg)


    def process_task_completion(self, msg):
        self.get_logger().info(f'Received task completion for task {msg.task_id} by robot {msg.robot_id}')

        transaction_id = self.get_transaction_id_by_task_id(msg.task_id)

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
                    self.robot_controller.update_robot_status(msg.robot_id, 'available')

                    self.reset_timer(msg.robot_id)  # Reset timer when all tasks are completed
                    self.allocate_transaction()

            self.publish_current_transactions(transaction_id)
        else:
            self.get_logger().warn(f'Transaction {transaction_id} not found in progress')
            
            
    # For GUI data
    def additional_task_completion_processing(self, msg):
        task_id = msg.task_id
        transaction_id = self.get_transaction_id_by_task_id(task_id)
        if transaction_id:
            self.publish_current_transactions(transaction_id)
        else:
            self.get_logger().warn(f'Task {task_id} completion received but not found in progress')
        

    def get_transaction_id_by_task_id(self, task_id):
        for transaction_id, (tasks, robot_id) in self.tasks_in_progress.items():
            for task in tasks:
                if task.task_id == task_id:
                    return transaction_id
        return None


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
