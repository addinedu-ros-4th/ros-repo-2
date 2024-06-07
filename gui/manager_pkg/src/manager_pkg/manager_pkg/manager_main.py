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
import pandas as pd
import cv2
from ament_index_python.packages import get_package_share_directory

# from task_msgs.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseWithCovarianceStamped 
from std_srvs.srv import SetBool
from std_msgs.msg import Empty
from task_msgs.msg import RobotStatus
from task_msgs.msg import PendingTaskList



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

class RobotStatusBarSubscriber(Node):
    def __init__(self, ui):
        super().__init__('robot_statusbar_subscriber')

        self.ui = ui
        self.ui.robot_status_clicked.connect(self.handle_statusbar)
        
        # 추가 예정 
        self.statusbar_sub = None

    def handle_statusbar(self, robot_name):
        self.robot_name = robot_name

    
class RobotStatusSubscriber(Node):
    def __init__(self, ui):
        super().__init__('robot_status_subscriber')

        self.ui = ui
        self.status_sub = self.create_subscription(RobotStatus, '/robot_status', self.status_callback, 10)
        
    def status_callback(self, data):
        self.ui.refresh_button.click()
        self.ui.refresh_button_2.click()

        self.robot_id = data.robot_id
        if self.robot_id == '91':
            self.robot_status1 = data.robot_status
            self.ui.status1.setText(self.robot_status1)
            self.update_button_style(self.ui.R1, self.robot_status1)

        elif  self.robot_id == '92':
            self.robot_status2 = data.robot_status
            self.ui.status2.setText(self.robot_status2)
            self.update_button_style(self.ui.R2, self.robot_status2)

        elif  self.robot_id == '93':
            self.robot_status3 = data.robot_status
            self.ui.status3.setText(self.robot_status3)
            self.update_button_style(self.ui.R3, self.robot_status3)


    def update_button_style(self, button, status):
        if status == "busy":
            button.setStyleSheet("background-color: rgb(246, 97, 81);""border-radius: 20px")
        elif status == "available":
            button.setStyleSheet("background-color: rgb(143, 240, 164);""border-radius: 20px")
        else:
            button.setStyleSheet("")

class RobotController(Node):
    def __init__(self, ui):
        super().__init__('robot_controller')   
        
        self.ui = ui
        self.ui.robot_control_clicked.connect(self.handle_control)
        self.ui.robot_stop_clicked.connect(self.robot_stop)
        self.emergency_pub1 = self.create_publisher(Empty, '/emergency_stop_1', 10)
        self.emergency_pub2 = self.create_publisher(Empty, '/emergency_stop_2', 10)
        self.emergency_pub3 = self.create_publisher(Empty, '/emergency_stop_3', 10)
        self.robot_name = None

    def handle_control(self, robot_name):
        self.robot_name = robot_name

    def robot_stop(self):
        if self.robot_name == 'robot_1':
            self.emergency_pub1.publish()

        elif self.robot_name == 'robot_2':
            self.emergency_pub2.publish()
        
        elif self.robot_name == 'robot_3':
            self.emergency_pub3.publish()

        else: 
            self.get_logger().info("robot_name invalid")


class PendingTaskSubscriber(Node):
    def __init__(self, ui):
        super().__init__('pending_task_node')
        
        self.ui = ui
        self.pending_task_sub = self.create_subscription(PendingTaskList, '/pending_tasks', self.pending_task_callback, 10)

    def pending_task_callback(self, data):
        tasks = []
        for task in data.tasks:
            tasks.append({
                'task_id': task.task_id,
                'bundle_id': task.bundle_id,
                'task_type': task.task_type,
                'location': task.location,
                'priority': task.priority
            })
        self.update_pending_tasks(tasks)

    def update_pending_tasks(self, tasks):
        self.ui.taskView.setRowCount(len(tasks))
        for row, task in enumerate(tasks):
            self.ui.taskView.setItem(row, 0, QTableWidgetItem(task['task_id']))
            self.ui.taskView.setItem(row, 1, QTableWidgetItem(task['bundle_id']))
            self.ui.taskView.setItem(row, 2, QTableWidgetItem(task['task_type']))
            self.ui.taskView.setItem(row, 3, QTableWidgetItem(task['location']))
            self.ui.taskView.setItem(row, 4, QTableWidgetItem(str(task['priority'])))

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

        # Set Timer
        timer = QTimer(self)
        timer.timeout.connect(self.update_map)
        timer.start(200)

        # Initialize pages
        self.init_main_page()
        self.init_robot_control_page()
        self.init_inbound_order_control_page()
        self.init_navigation_buttons()
        self.init_ros2_node()
        
        # Set up a timer to update stock info every 5 seconds
        self.stock_update_timer = QTimer(self)
        self.stock_update_timer.timeout.connect(self.update_stock_info)
        self.stock_update_timer.start(5000)  # Update every 5000 milliseconds (5 seconds)
        print("Timer started for updating stock info every 5 seconds")


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
        home_png_path = os.path.join(get_package_share_directory('manager_pkg'), 'image', 'home.png')
        robot_png_path = os.path.join(get_package_share_directory('manager_pkg'), 'image', 'robot.png')
        list_png_path = os.path.join(get_package_share_directory('manager_pkg'), 'image', 'list.png')
        self.set_button_icon(self.home, home_png_path)
        self.set_button_icon(self.robot, robot_png_path)
        self.set_button_icon(self.list, list_png_path)
        self.set_button_icon(self.home_2, home_png_path)
        self.set_button_icon(self.robot_2, robot_png_path)
        self.set_button_icon(self.list_2, list_png_path)
        self.set_button_icon(self.home_3, home_png_path)
        self.set_button_icon(self.robot_3, robot_png_path)
        self.set_button_icon(self.list_3, list_png_path)

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
        self.R1.clicked.connect(lambda: self.switch_page(1, 0))
        self.R2.clicked.connect(lambda: self.switch_page(1, 1))
        self.R3.clicked.connect(lambda: self.switch_page(1, 2))

    def set_button_icon(self, button, icon_path):
        icon = QIcon(icon_path)
        button.setIcon(icon)
        button.setIconSize(QSize(35, 35))  # Adjust the size as needed

    def switch_page(self, page_index, robot_index=None):
        self.stackedWidget.setCurrentIndex(page_index)
        if robot_index is not None:
            self.robotComboBox.setCurrentIndex(robot_index)


    def init_main_page(self):
    #     # Main Page: Real-time location of robots, Task list, Current Stock info
        self.map_label = self.findChild(QLabel, 'mapLabel')  # Assuming there's a QLabel for the map
        self.update_stock_info()

        robotstatus = self.db_manager.fetch_all_product("RobotStatus")

        df = pd.DataFrame(robotstatus, columns=['robot_id', 'status'])
        id_list = df['robot_id'].tolist()
        status_list = df['status'].tolist()

        self.update_robot_button(self.R1, self.status1, id_list[0], status_list[0])
        self.update_robot_button(self.R2, self.status2, id_list[1], status_list[1])
        self.update_robot_button(self.R3, self.status3, id_list[2], status_list[2])


    def update_robot_button(self, button, status_label, robot_id, status):
        button.setText(robot_id)
        status_label.setText(status)
        if status == "busy":
            button.setStyleSheet("background-color: rgb(246, 97, 81);""border-radius: 20px")
        elif status == "available":
            button.setStyleSheet("background-color: rgb(143, 240, 164);""border-radius: 20px")
        else:
            button.setStyleSheet("")
    
    def init_ros2_node(self):
        pass

    def update_map(self):
        # timer 이용해서 amcl값 map에 업데이트하기.
        # 3기 코드 OR 동규 코드 참고
        pass


    def update_task_list(self):
        pass

    def update_stock_info(self):

        product_inventory = self.db_manager.fetch_all_product("ProductInventory")
        product_info = self.db_manager.get_data("ProductInfo", ["item_id", "item_tag"])

        df_inventory = pd.DataFrame(product_inventory, columns=['item_id', 'item_name', 'stock'])
        df_info = pd.DataFrame(product_info, columns=(["item_id", "item_tag"]))

        df_merged = pd.merge(df_inventory, df_info, on='item_id')

        self.A1Label = self.findChild(QLabel, 'A1Label')
        self.A2Label = self.findChild(QLabel, 'A2Label')
        self.B1Label = self.findChild(QLabel, 'B1Label')
        self.B2Label = self.findChild(QLabel, 'B2Label')
        self.C1Label = self.findChild(QLabel, 'C1Label')
        self.C2Label = self.findChild(QLabel, 'C2Label')

        label_mapping = {
        1: self.A1Label,
        2: self.A2Label,
        3: self.B1Label,
        4: self.B2Label,
        5: self.C1Label,
        6: self.C2Label,
        }
        
        for index, row in df_merged.iterrows():
            item_id = row['item_id']
            item_name = row['item_name']
            stock = row['stock']
            item_tag = row['item_tag']

            if item_id in label_mapping:
                self.label = label_mapping[item_id]
                self.label.setText(f"{item_tag}\n{item_name}\nstock: {stock}")
        
    def update_product_quantity(self, product_id, quantity):
        query = "UPDATE ProductInventory SET stock = %s WHERE item_id = %s"
        self.db_manager.cursor.execute(query, (quantity, product_id))
        self.db_manager.conn.commit()
        print(f"Updated product {product_id} with quantity {quantity}")

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
        self.OrderList = self.findChild(QTableWidget, 'OrderList')

        
        self.scanned_data = ""
        self.scan_button = self.findChild(QPushButton, 'scan_button')  # Ensure this matches the object name in your UI
        self.scan_button.clicked.connect(self.scan_barcode)
        
        self.refresh_button = self.findChild(QPushButton, 'refresh_button')  # Ensure this matches the object name in your UI
        self.refresh_button.clicked.connect(self.update_inbound_list)

        self.barcode_scanner = BarcodeScanner()  # Properly initialize BarcodeScanner
        self.barcode_scanner.barcode_scanned.connect(self.update_inbound_list)  # Connect signal to slot

        self.refresh_button_2 = self.findChild(QPushButton, 'refresh_button_2')
        self.refresh_button_2.clicked.connect(self.update_order_list)


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
        order_list = self.db_manager.fetch_all_product("ProductOrder")

        df = pd.DataFrame(order_list, columns=['order_id', 'user_id', 'item_id', 'item_name', 'quantities', 'order_time'])

        # tableWidget 업데이트
        self.OrderList.setRowCount(len(df))
        self.OrderList.setColumnCount(len(df.columns))
        self.OrderList.setHorizontalHeaderLabels(df.columns)

        for row_index, row in enumerate(df.itertuples(index=False)):
            for col_index, value in enumerate(row):
                item = QTableWidgetItem(str(value))
                self.OrderList.setItem(row_index, col_index, item)

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
    statusbar_node = RobotStatusBarSubscriber(window)
    status_node = RobotStatusSubscriber(window)
    pending_task_node = PendingTaskSubscriber(window)
    control_node = RobotController(window)

    executor.add_node(amcl_node)
    executor.add_node(picam_node)
    executor.add_node(statusbar_node)
    executor.add_node(status_node)
    executor.add_node(pending_task_node)
    executor.add_node(control_node)

    thread = Thread(target=executor.spin)
    thread.start()

    app.exec_()

if __name__ == '__main__':
    main()