import sys, os

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5 import uic

from data_manager.database_manager import DatabaseManager
from manager_pkg.barcode_scanner import BarcodeScanner
import rclpy
from rclpy.node import Node
from threading import Thread
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile, qos_profile_sensor_data
import numpy as np
import cv2
from ament_index_python.packages import get_package_share_directory

# from task_msgs.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseWithCovarianceStamped 
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from task_msgs.msg import RobotStatus



global amcl_1, amcl_2, amcl_3
amcl_1 = PoseWithCovarianceStamped()
amcl_2 = PoseWithCovarianceStamped()
amcl_3 = PoseWithCovarianceStamped()

class AmclSubscriber(Node):
    def __init__(self):
        super().__init__('amcl_subscriber')

        self.pose1_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose_1', self.amcl_callback1, 10)
        self.pose2_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose_2', self.amcl_callback2, 10)
        self.pose3_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose_3', self.amcl_callback3, 10)

    def amcl_callback1(self, amcl):
        global amcl_1
        amcl_1 = amcl
        
    def amcl_callback2(self, amcl):
        global amcl_2
        amcl_2 = amcl
        
    def amcl_callback3(self, amcl):
        global amcl_3
        amcl_3 = amcl
    
class PiCamSubscriber(Node):
    def __init__(self, ui):
        super().__init__('pi_cam_sub_scriber') 
        
        self.ui = ui
        self.ui.robot_picam_clicked.connect(self.handle_picam)
        
        self.cam_client1 = self.create_client(SetBool, "/camera_control_1")
        self.cam_client2 = self.create_client(SetBool, "/camera_control_2")
        self.cam_client3 = self.create_client(SetBool, "/camera_control_3")
        self.picam_sub1 = None
        self.picam_sub2 = None
        self.picam_sub3 = None 
        self.robot_name = None

    def handle_picam(self, robot_name):
        if robot_name == 'robot_1':
            self.picam_sub1 = self.create_subscription(CompressedImage, '/camera/compressed_1', self.img_callback1, 10)
            self.cam_client1.call_async(True)
            self.robot_name = robot_name
            
            if self.picam_sub2:
                self.picam_sub2.destroy_subscription()
                self.cam_client2.call_async(False)
            
            if self.picam_sub3:
                self.picam_sub3.destroy_subscription()
                self.cam_client3.call_async(False)

            
        elif robot_name == 'robot_2':
            self.picam_sub2 = self.create_subscription(CompressedImage, '/camera/compressed_2', self.img_callback2, 10)
            self.cam_client2.call_async(True)
            self.robot_name = robot_name

            if self.picam_sub1:
                self.picam_sub1.destroy_subscription()
                self.cam_client1.call_async(False)

            if self.picam_sub3:
                self.picam_sub3.destroy_subscription()
                self.cam_client3.call_async(False)

            
        elif robot_name == 'robot_3':
            self.picam_sub3 = self.create_subscription(CompressedImage, '/camera/compressed_3', self.img_callback3, 10)
            self.cam_client3.call_async(True)
            self.robot_name = robot_name

            if self.picam_sub1:
                self.picam_sub1.destroy_subscription()
                self.cam_client1.call_async(False)

            if self.picam_sub2:
                self.picam_sub2.destroy_subscription()
                self.cam_client2.call_async(False)

        else:
            self.get_logger().info("robot_name invalid")


    def img_callback1(self, data):
        if self.robot_name == 'robot_1':
            np_arr = np.frombuffer(data.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
            height, width, channel = image_np.shape
            bytes_per_line = 3 * width
            q_image = QImage(image_np.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_image)
            self.ui.picamLabel.setPixmap(pixmap)
        
        else:
            pass
        
    def img_callback2(self, data):
        if self.robot_name == 'robot_2':
            np_arr = np.frombuffer(data.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
            height, width, channel = image_np.shape
            bytes_per_line = 3 * width
            q_image = QImage(image_np.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_image)
            self.ui.picamLabel.setPixmap(pixmap)
        
        else:
            pass
    
    def img_callback3(self, data):
        if self.robot_name == 'robot_3':
            np_arr = np.frombuffer(data.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
            height, width, channel = image_np.shape
            bytes_per_line = 3 * width
            q_image = QImage(image_np.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_image)
            self.ui.picamLabel.setPixmap(pixmap)
        
        else:
            pass

class RobotStatusSubscriber(Node):
    def __init__(self, ui):
        super().__init__('robot_status_subscriber')

        self.ui = ui
        self.ui.robot_status_clicked.connect(self.handle_status)

        self.status_sub1 = self.create_subscription(RobotStatus, '/robot_status_1', self.status_callback1, 10)
        self.status_sub2 = self.create_subscription(RobotStatus, '/robot_status_2', self.status_callback2, 10)
        self.status_sub3 = self.create_subscription(RobotStatus, '/robot_status_3', self.status_callback3, 10) 
        self.robot_name = None

    def handle_status(self, robot_name):
        self.robot_name = robot_name

    def status_callback1(self, data):
        if self.robot_name == 'robot_1':
            self.ui.statusLabel.setText(data)

        else:
            pass

    def status_callback2(self, data):
        if self.robot_name == 'robot_2':
            self.ui.statusLabel.setText(data)

        else:
            pass

    def status_callback3(self, data):
        if self.robot_name == 'robot_3':
            self.ui.statusLabel.setText(data)

        else:
            pass

class RobotController(Node):
    def __init__(self, ui):
        super().__init__('robot_controller')   
        
        self.ui = ui
        self.ui.robot_control_clicked.connect(self.handle_control)
        self.ui.robot_stop_clicked.connect(self.robot_stop)
        self.control_pub1 = self.create_publisher(Twist, 'base_controller/cmd_vel_unstamped_1', 10)
        self.control_pub2 = self.create_publisher(Twist, 'base_controller/cmd_vel_unstamped_2', 10)
        self.control_pub3 = self.create_publisher(Twist, 'base_controller/cmd_vel_unstamped_3', 10)
        
        self.Twist = Twist()
        self.robot_name = None

    def handle_control(self, robot_name):
        self.robot_name = robot_name

    def robot_stop(self):
        if self.robot_name == 'robot_1':
            self.Twist.linear.x = 0.0
            self.Twist.angular.z = 0.0
            self.control_pub1.publish(self.Twist)

        elif self.robot_name == 'robot_2':
            self.Twist.linear.x = 0.0
            self.Twist.angular.z = 0.0
            self.control_pub1.publish(self.Twist)
        
        elif self.robot_name == 'robot_3':
            self.Twist.linear.x = 0.0
            self.Twist.angular.z = 0.0
            self.control_pub1.publish(self.Twist)

        else: 
            self.get_logger().info("robot_name invalid")


class Ui_MainWindow(QMainWindow):
    robot_picam_clicked = pyqtSignal(str)
    robot_status_clicked = pyqtSignal(str)
    robot_control_clicked = pyqtSignal(str)
    robot_stop_clicked = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Manager Window")

        self.db_manager = DatabaseManager(host='localhost')
        self.db_manager.connect_database()
        self.db_manager.create_table()

        # Load UI
        ui_path = os.path.join(get_package_share_directory('manager_pkg'), 'ui', 'manager.ui')
        uic.loadUi(ui_path, self)

        # Initialize the Stacked Widget
        self.stackedWidget = self.findChild(QStackedWidget, 'stackedWidget')

        # Initialize pages
        self.init_main_page()
        self.init_robot_control_page()
        self.init_inbound_order_control_page()

        self.init_navigation_buttons()

        self.init_ros2_node()


    def init_navigation_buttons(self):
        # Find buttons
        self.home = self.findChild(QPushButton, 'home')
        self.robot = self.findChild(QPushButton, 'robot')
        self.list = self.findChild(QPushButton, 'list')
        self.home_2 = self.findChild(QPushButton, 'home_2')
        self.robot_2 = self.findChild(QPushButton, 'robot_2')
        self.list_2 = self.findChild(QPushButton, 'list_2')
        self.home_3 = self.findChild(QPushButton, 'home_3')
        self.robot_3 = self.findChild(QPushButton, 'robot_3')
        self.list_3 = self.findChild(QPushButton, 'list_3')

        # Set icons for buttons
        self.set_button_icon(self.home, 'gui/manager/image/home.png')
        self.set_button_icon(self.robot, 'gui/manager/image/robot.png')
        self.set_button_icon(self.list, 'gui/manager/image/list.png')
        self.set_button_icon(self.home_2, 'gui/manager/image/home.png')
        self.set_button_icon(self.robot_2, 'gui/manager/image/robot.png')
        self.set_button_icon(self.list_2, 'gui/manager/image/list.png')
        self.set_button_icon(self.home_3, 'gui/manager/image/home.png')
        self.set_button_icon(self.robot_3, 'gui/manager/image/robot.png')
        self.set_button_icon(self.list_3, 'gui/manager/image/list.png')

        # Connect buttons to switch pages
        self.home.clicked.connect(lambda: self.switch_page(0))
        self.robot.clicked.connect(lambda: self.switch_page(1))
        self.list.clicked.connect(lambda: self.switch_page(2))
        self.home_2.clicked.connect(lambda: self.switch_page(0))
        self.robot_2.clicked.connect(lambda: self.switch_page(1))
        self.list_2.clicked.connect(lambda: self.switch_page(2))
        self.home_3.clicked.connect(lambda: self.switch_page(0))
        self.robot_3.clicked.connect(lambda: self.switch_page(1))
        self.list_3.clicked.connect(lambda: self.switch_page(2))

    def set_button_icon(self, button, icon_path):
        icon = QIcon(icon_path)
        button.setIcon(icon)
        button.setIconSize(QSize(35, 35))  # Adjust the size as needed

    def switch_page(self, page_index):
        self.stackedWidget.setCurrentIndex(page_index)

    def init_main_page(self):
        # Main Page: Real-time location of robots, Task list, Current Stock info
        self.map_label = self.findChild(QLabel, 'mapLabel')  # Assuming there's a QLabel for the map

    def init_ros2_node(self):
        # rclpy.init()
        # self.executor = MultiThreadedExecutor()
        # self.amcl_subscriber = AmclSubscriber()
        # self.executor.add_node(self.amcl_subscriber)

        # # Run ROS 2 executor in a separate thread
        # self.ros_thread = Thread(target=self.executor.spin)
        # self.ros_thread.start()

        # # Timer to update the map periodically
        # self.timer = QTimer(self)
        # self.timer.timeout.connect(self.update_map)
        # self.timer.start(1000)  # Update the map every second
        pass

    def update_map(self):
        # # Fetch the latest positions
        # global amcl_1, amcl_2, amcl_3
        # positions = [amcl_1, amcl_2, amcl_3]
        
        # # Placeholder for actual map update logic
        # for idx, pose in enumerate(positions):
        #     if pose:
        #         position = pose.pose.pose.position
        #         orientation = pose.pose.pose.orientation
        #         print(f"Robot {idx + 1} Position: {position.x}, {position.y}, Orientation: {orientation.z}")
        
        # # Update your QLabel or map with the positions
        # self.map_label.setPixmap(updated_map_pixmap)
        pass

    def update_task_list(self):
        pass

    def update_stock_info(self):
        pass

    def init_robot_control_page(self):
        self.robotComboBox = self.findChild(QComboBox, 'robotComboBox')
        self.picamLabel = self.findChild(QLabel, 'picamLabel')
        self.statusLabel = self.findChild(QLabel, 'statusLabel')
        self.btnStop = self.findChild(QPushButton, 'btnStop')

        self.robotComboBox.addItems(["robot_1", "robot_2", "robot_3"])
        self.robotComboBox.currentIndexChanged.connect(self.update_robot_info)
        self.btnStop.clicked.connect(self.robot_stop)
        # Initialize with the first robot's data
        self.update_robot_info(0)

    def update_robot_info(self, index):
        robot_name = self.robotComboBox.itemText(index)
        self.robot_picam_clicked.emit(robot_name)
        self.robot_control_clicked.emit(robot_name)
        self.robot_status_clicked.emit(robot_name)

    def robot_stop(self):
        self.robot_stop_clicked.emit(True)


    def init_inbound_order_control_page(self):
        self.inbound_list = self.findChild(QTableWidget, 'inbound_list')  # Ensure this matches the object name in your UI
        self.scanned_data = ""
        self.scan_button = self.findChild(QPushButton, 'scan_button')  # Ensure this matches the object name in your UI
        self.scan_button.clicked.connect(self.scan_barcode)
        
        self.refresh_button = self.findChild(QPushButton, 'refresh_button')  # Ensure this matches the object name in your UI
        self.refresh_button.clicked.connect(self.update_inbound_list)

        self.barcode_scanner = BarcodeScanner()  # Properly initialize BarcodeScanner
        self.barcode_scanner.barcode_scanned.connect(self.update_inbound_list)  # Connect signal to slot

    def update_inbound_list(self, data=None):
        # Clear the existing rows in the QTableWidget
        self.inbound_list.setRowCount(0)
        self.inbound_list.setColumnCount(5)
        self.inbound_list.setHorizontalHeaderLabels(["Item Name", "Quantity", "Inbound Zone", "Arrival Date", "Status"])

        # Create a new cursor and fetch all rows from the Inbound table
        self.db_manager.connect_database()  # Reconnect to refresh the cursor
        all_rows = self.db_manager.get_data("Inbound", ["item_name", "quantity", "inbound_zone", "arrival_date", "current_status"])
        
        # Populate the QTableWidget with data from the Inbound table
        for row in all_rows:
            row_position = self.inbound_list.rowCount()
            self.inbound_list.insertRow(row_position)
            for column, value in enumerate(row):
                self.inbound_list.setItem(row_position, column, QTableWidgetItem(str(value)))
        print("Updated inbound_list with new data")  # Debugging print statement

    def update_order_list(self):
        pass

    def scan_barcode(self):
        import threading
        threading.Thread(target=self.barcode_scanner.append_list).start()
        

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    app = QApplication(sys.argv)

    window = Ui_MainWindow()
    window.show()

    amcl_node = AmclSubscriber()
    picam_node = PiCamSubscriber(window)
    status_node = RobotStatusSubscriber(window)
    control_node = RobotController(window)


    executor.add_node(amcl_node)
    executor.add_node(picam_node)
    executor.add_node(status_node)
    executor.add_node(control_node)

    thread = Thread(target=executor.spin)
    thread.start()

    app.exec_()

if __name__ == '__main__':
    main()