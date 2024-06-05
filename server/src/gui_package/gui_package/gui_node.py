import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel
from PyQt5.QtCore import QMetaObject, Qt, Q_ARG, pyqtSlot
import rclpy
from rclpy.node import Node
from task_msgs.msg import TaskList

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("gui_node")

        self.layout = QVBoxLayout()
        self.widget = QWidget()
        self.widget.setLayout(self.layout)
        self.setCentralWidget(self.widget)

    @pyqtSlot(list)
    def update_pending_tasks(self, tasks):
        # Clear the layout
        for i in reversed(range(self.layout.count())):
            self.layout.itemAt(i).widget().setParent(None)
        # Add new tasks
        for task in tasks:
            task_label = QLabel(f"Task ID: {task['task_id']}, Bundle ID: {task['bundle_id']}, Task Type: {task['task_type']}, Location: {task['location']}, Priority: {task['priority']}")
            self.layout.addWidget(task_label)

class GuiNode(Node):
    def __init__(self, window):
        super().__init__('gui_node')
        self.window = window
        self.subscription = self.create_subscription(TaskList, '/unassigned_tasks', self.pending_task_callback, 10)

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
        # Use QMetaObject to safely update the GUI from the ROS2 callback
        QMetaObject.invokeMethod(self.window, "update_pending_tasks", Qt.QueuedConnection, Q_ARG(list, tasks))

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    gui_node = GuiNode(window)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(gui_node)

    def run_executor():
        executor.spin()

    # Run the executor in a separate thread
    import threading
    executor_thread = threading.Thread(target=run_executor, daemon=True)
    executor_thread.start()

    try:
        sys.exit(app.exec_())
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
