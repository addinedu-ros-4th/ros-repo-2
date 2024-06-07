import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5.QtCore import QThread
from task_msgs.msg import TaskCompletion
import rclpy
import json

class TaskSubscriber(Node, QObject):
    current_transactions_signal = pyqtSignal(list)

    def __init__(self):
        Node.__init__(self, 'transactions_subscriber')
        QObject.__init__(self)
        self.subscription_current = self.create_subscription(String, '/current_transactions', self.current_transaction_callback, 10)
        self.subscription_complete = self.create_subscription(TaskCompletion, '/task_completion', self.task_complete_callback, 10)
        self.transactions = []
        
    def current_transaction_callback(self, msg):
        transactions = json.loads(msg.data)
        self.current_transactions_signal.emit(transactions)
        
        
    def task_complete_callback(self, msg):
        # TaskCompletion 메시지를 받아서 transactions 리스트를 업데이트
        for transaction in self.transactions:
            for task in transaction['tasks']:
                if task['task_id'] == msg.task_id:
                    task['completed'] = msg.success
        self.current_transactions_signal.emit(self.transactions)
    
    
    def get_transactions(self):
        return self.transactions


class TaskThread(QThread):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

    def run(self):
        rclpy.spin(self.ros_node)
