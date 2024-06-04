import sys
sys.path.append('./db/src')  # DatabaseManager.py 파일의 경로를 추가

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5 import uic
from DatabaseManager import DatabaseManager
from barcode_scanner import BarcodeScanner
import rclpy
from rclpy.node import Node
from threading import Thread
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile, qos_profile_sensor_data

# from task_msgs.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseWithCovarianceStamped 



# class AmclSubscriber(Node):

#     def __init__(self):

#         super().__init__('amcl_subscriber')
  
#         amcl_pose_qos = QoSProfile(
#                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
#                 reliability=QoSReliabilityPolicy.RELIABLE,
#                 history=QoSHistoryPolicy.KEEP_LAST,
#                 depth=1)
        
#         # 3개의 로봇 위치 표시
#         self.pose1 = self.create_subscription(
#             PoseWithCovarianceStamped, 
#             '/amcl_pose_1', 
#             self.amcl_callback1, 
#             amcl_pose_qos)
        
#         self.pose2 = self.create_subscription(
#             PoseWithCovarianceStamped, 
#             '/amcl_pose_2', 
#             self.amcl_callback2, 
#             amcl_pose_qos)
        
#         self.pose3 = self.create_subscription(
#             PoseWithCovarianceStamped, 
#             '/amcl_pose_3', 
#             self.amcl_callback3, 
#             amcl_pose_qos)

#     def amcl_callback1(self, amcl):
#         global amcl_1
#         amcl_1 = amcl
        
#     def amcl_callback2(self, amcl):
#         global amcl_2
#         amcl_2 = amcl
        
#     def amcl_callback3(self, amcl):
#         global amcl_3
#         amcl_3 = amcl


class Ui_MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Manager Window")

        self.db_manager = DatabaseManager(host='localhost')
        self.db_manager.connect_database()
        self.db_manager.create_table()

        # Load UI
        uic.loadUi("gui/manager/ui/manager.ui", self)

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

        self.robotComboBox.addItems(["robot_1", "robot_2", "robot_3"])
        self.robotComboBox.currentIndexChanged.connect(self.update_robot_info)

        # Initialize with the first robot's data
        self.update_robot_info(0)

    def update_robot_info(self, index):
        robot_name = self.robotComboBox.itemText(index)
        self.display_robot_picam(robot_name)
        self.display_robot_status(robot_name)

    def display_robot_picam(self, robot_name):
        # # Replace with the actual path to the PiCam feed images or stream
        # picam_path = f'path/to/{robot_name}_picam_feed.jpg'
        # pixmap = QPixmap(picam_path)
        # self.picamLabel.setPixmap(pixmap)
        # self.picamLabel.setScaledContents(True)  # Ensure the image scales to the label size
        pass

    def display_robot_status(self, robot_name):
        # # Replace with the actual logic to fetch the robot's status
        # status = self.db_manager.get_robot_status(robot_name)
        # self.statusLabel.setText(f"Status: {status}")
        pass

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
        

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Ui_MainWindow()
    window.show()
    sys.exit(app.exec_())
