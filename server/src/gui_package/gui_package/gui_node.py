import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QTableWidget, QTableWidgetItem, QWidget
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from task_msgs.msg import PendingTaskList

class GUI(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.pending_task_sub = self.create_subscription(PendingTaskList, '/pending_tasks', self.pending_task_callback, 10)

        self.app = QApplication(sys.argv)
        self.window = ManagerGUI()
        self.window.show()


    def pending_task_callback(self, msg):
        tasks = []
        for task in msg.tasks:
            tasks.append({
                'task_id': task.task_id,
                'bundle_id': task.bundle_id,
                'task_type': task.task_type,
                'location': task.location,
                'priority': task.priority
            })
        self.get_logger().info(f"Received pending tasks: {tasks}")
        self.window.update_pending_tasks(tasks)

    def run(self):
        self.app.exec_()


class ManagerGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle('Task Manager')
        self.setGeometry(100, 100, 800, 600)

        self.pending_task_table = QTableWidget()
        self.pending_task_table.setColumnCount(5)
        self.pending_task_table.setHorizontalHeaderLabels(['Task ID', 'Bundle ID', 'Task Type', 'Location', 'Priority'])

        layout = QVBoxLayout()
        layout.addWidget(self.robot_status_table)
        layout.addWidget(self.inventory_status_table)
        layout.addWidget(self.pending_task_table)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    def update_pending_tasks(self, tasks):
        self.get_logger().info(f"Updating GUI with pending tasks: {tasks}")
        self.pending_task_table.setRowCount(len(tasks))
        for row, task in enumerate(tasks):
            self.pending_task_table.setItem(row, 0, QTableWidgetItem(task['task_id']))
            self.pending_task_table.setItem(row, 1, QTableWidgetItem(task['bundle_id']))
            self.pending_task_table.setItem(row, 2, QTableWidgetItem(task['task_type']))
            self.pending_task_table.setItem(row, 3, QTableWidgetItem(task['location']))
            self.pending_task_table.setItem(row, 4, QTableWidgetItem(str(task['priority'])))

def main(args=None):
    rclpy.init(args=args)
    gui_node = GUI()

    executor = MultiThreadedExecutor()
    executor.add_node(gui_node)

    try:
        gui_node.run()
        executor.spin()
    finally:
        executor.shutdown()
        gui_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
