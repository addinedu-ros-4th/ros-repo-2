import threading
import rclpy
from order_receiver import OrderReceiver
from task_allocator import TaskAllocator
from robot_status_monitor import RobotStatusMonitor
from task_success_handler import TaskSuccessHandler

def main():
    rclpy.init()

    order_receiver = OrderReceiver()
    task_allocator = TaskAllocator()
    robot_status_monitor = RobotStatusMonitor()
    task_success_handler = TaskSuccessHandler()

    nodes = [order_receiver, task_allocator, robot_status_monitor, task_success_handler]

    threads = []
    for node in nodes:
        thread = threading.Thread(target=rclpy.spin, args=(node,))
        thread.start()
        threads.append(thread)

    try:
        for thread in threads:
            thread.join()
    except KeyboardInterrupt:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
