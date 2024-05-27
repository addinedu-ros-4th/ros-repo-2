import unittest
from unittest.mock import MagicMock, patch
from task_allocator import TaskAllocator, PriorityTask
from task_msgs.msg import TaskList, Task, RobotStatus, TaskCompletion
from task_msgs.srv import AllocateTask

class TestTaskAllocator(unittest.TestCase):

    def setUp(self):
        self.node = TaskAllocator()
        
        # Mock methods that interact with ROS2
        self.node.get_logger = MagicMock()
        self.node.create_subscription = MagicMock()
        self.node.create_client = MagicMock()
        
        # Mock the task and robot status subscriptions
        self.node.task_list_sub = MagicMock()
        self.node.robot_status_sub = MagicMock()
        self.node.completion_sub = MagicMock()
        
        # Prepare mock responses for service calls
        self.mock_allocate_task_client = MagicMock()
        self.node.create_client.return_value = self.mock_allocate_task_client

    def test_task_list_callback(self):
        # Create a mock TaskList message
        task_list_msg = TaskList()
        task1 = MagicMock()
        task1.task_id = '1'
        task1.task_type = 'outbound'
        task1.priority = 1
        task1.bundle_id = 'bundle1'
        task1.item = 'ramen'
        task1.quantity = 2
        task1.location = 'A1'
        
        task2 = MagicMock()
        task2.task_id = '2'
        task2.task_type = 'outbound'
        task2.priority = 1
        task2.bundle_id = 'bundle1'
        task1.item = 'cola'
        task1.quantity = 2
        task1.location = 'A2'
        
        task_list_msg.tasks = [task1, task2]
        
        # Call the callback function
        self.node.task_list_callback(task_list_msg)
        
        # Verify the tasks were added correctly
        self.assertEqual(len(self.node.tasks), 2)

    def test_allocate_tasks(self):
        # Prepare robot status
        self.node.robot_status = {
            'robotA': 'available',
            'robotB': 'available'
        }
        
        # Prepare tasks in the priority queue
        task1 = MagicMock()
        task1.task_id = '1'
        task1.task_type = 'outbound'
        task1.priority = 1
        task1.bundle_id = 'bundle1'
        task1.item = 'ramen'
        task1.quantity = 2
        task1.location = 'A1'
        
        task2 = MagicMock()
        task2.task_id = '2'
        task2.task_type = 'outbound'
        task2.priority = 1
        task2.bundle_id = 'bundle1'
        task1.item = 'cola'
        task1.quantity = 2
        task1.location = 'A2'
        
        self.node.tasks = [
            PriorityTask(priority=1, task=[task1]),
            PriorityTask(priority=1, task=[task1])
        ]
        
        # Call the allocate_tasks function
        self.node.allocate_tasks()
        
        # Verify the tasks were assigned correctly
        self.assertEqual(self.node.robot_status['robotA'], 'handling_inbound')
        self.assertEqual(self.node.robot_status['robotB'], 'handling_outbound')
        self.assertEqual(len(self.node.tasks), 0)

    def test_task_completion_callback(self):
        # Prepare a task completion message
        completion_msg = TaskCompletion()
        completion_msg.task_id = '1'
        completion_msg.robot_id = 'robot_213'
        completion_msg.success = True
        
        # Prepare tasks in progress
        task1 = MagicMock()
        task1.task_id = '1'
        task1.bundle_id = 'bundle1'
        
        self.node.tasks_in_progress = {
            '1_robot_213': ([task1], 'robot_213')
        }
        
        # Call the callback function
        self.node.task_completion_callback(completion_msg)
        
        # Verify the tasks were completed correctly
        self.assertNotIn('1_robot_213', self.node.tasks_in_progress)
        self.assertEqual(self.node.robot_status['robot_213'], 'available')

    def test_assign_specific_task_to_robot(self):
        # Prepare task and robot_id
        task = MagicMock()
        task.task_id = '1'
        task.task_type = 'inbound'
        task.priority = 1
        task.bundle_id = 'bundle1'
        
        robot_id = 'robot_213'
        transaction_id = '1_robot_213'
        
        # Mock the future response
        future = MagicMock()
        future.result.return_value.success = True
        
        self.mock_allocate_task_client.call_async.return_value = future
        
        # Call the function
        self.node.assign_specific_task_to_robot(task, robot_id, transaction_id)
        
        # Verify the service was called with the correct parameters
        self.mock_allocate_task_client.call_async.assert_called_once()
        self.node.task_allocation_response(future, task, robot_id, transaction_id)

if __name__ == '__main__':
    unittest.main()
