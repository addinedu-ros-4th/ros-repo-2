import unittest
from unittest.mock import MagicMock
from std_msgs.msg import String
from task_msgs.msg import TaskList, Task
from task_manager.order_receiver import OrderReceiver

class TestOrderReceiver(unittest.TestCase):
    def setUp(self):
        self.node = OrderReceiver()

        # Mock methods that interact with ROS2
        self.node.get_logger = MagicMock()
        self.node.create_publisher = MagicMock()
        self.node.create_subscription = MagicMock()

        # Mock the publisher
        self.mock_task_list_publisher = MagicMock()
        self.node.create_publisher.return_value = self.mock_task_list_publisher

    def test_order_callback_valid(self):
        # Mock valid JSON message
        valid_msg = String()
        valid_msg.data = json.dumps([
            {"user_id": 1, "item_name": ["cola", "water"], "quantities": [2, 3]}
        ])

        # Call the callback function with the mock message
        self.node.order_callback(valid_msg)

        # Verify the tasks were added and published correctly
        self.assertEqual(len(self.node.order_list), 1)
        self.assertEqual(len(self.node.order_list[0]['items']), 2)
        self.mock_task_list_publisher.publish.assert_called_once()

    def test_order_callback_invalid(self):
        # Mock invalid JSON message
        invalid_msg = String()
        invalid_msg.data = '{"user_id": 1, "item_name": "cola", "quantities": 2}'  # Not a list

        # Call the callback function with the mock message
        self.node.order_callback(invalid_msg)

        # Verify the invalid format was logged
        self.node.get_logger().warn.assert_called_with("order_info is not a list")

    def test_process_single_order_missing_key(self):
        # Mock single order with missing key
        single_order = {"user_id": 1, "item_name": ["cola"], "quantities": [2]}

        # Call process_single_order with missing key
        del single_order["quantities"]

        self.node.process_single_order(single_order)

        # Verify the error was logged
        self.node.get_logger().error.assert_called()

if __name__ == '__main__':
    unittest.main()
