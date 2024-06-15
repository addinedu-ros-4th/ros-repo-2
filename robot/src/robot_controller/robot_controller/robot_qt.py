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
import json
from ament_index_python.packages import get_package_share_directory
import yaml

# from task_msgs.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseWithCovarianceStamped 
from std_srvs.srv import SetBool
from std_msgs.msg import Empty
from task_msgs.msg import RobotStatus
from task_msgs.msg import TaskList
from manager_pkg.transactions_subscriber import TaskSubscriber,TaskThread



global amcl_1, amcl_2, amcl_3
amcl_1 = PoseWithCovarianceStamped()
amcl_2 = PoseWithCovarianceStamped()
amcl_3 = PoseWithCovarianceStamped()

#추가
map_yaml_file = os.path.join(get_package_share_directory('manager_pkg'), 'map', 're_map.yaml')

class AmclSubscriber(Node):
    def __init__(self):
        super().__init__('amcl_subscriber')

        self.pose1_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose_1', self.amcl_callback1, 10)
        self.pose2_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose_2', self.amcl_callback2, 10)
        # self.pose3_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose_3', self.amcl_callback3, 10)

    def amcl_callback1(self, amcl):
        global amcl_1
        amcl_1 = amcl
        
    def amcl_callback2(self, amcl):
        global amcl_2
        amcl_2 = amcl
        
    # def amcl_callback3(self, amcl):
    #     global amcl_3
    #     amcl_3 = amcl




class Ui_MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Manager Window")

        # Load UI
        ui_path = os.path.join(get_package_share_directory('manager_pkg'), 'ui', 'manager.ui')
        uic.loadUi(ui_path, self)

        # Initialize the Stacked Widget
        self.stackedWidget = self.findChild(QStackedWidget, 'stackedWidget')
        self.stackedWidget.setCurrentIndex(0)

        # Initialize pages
        self.init_main_page()
        self.init_map()

        # Set Timer
        self.map_timer = QTimer(self)
        self.map_timer.timeout.connect(self.update_map)
        self.map_timer.start(200)


    def init_map(self):
        with open(map_yaml_file) as f:
            self.map_yaml_data = yaml.full_load(f)

        self.image_scale = 1.5
        self.pixmap = QPixmap(os.path.join(get_package_share_directory('manager_pkg'), 'map', self.map_yaml_data['image']))
        self.scaled_pixmap = self.pixmap.scaled(int(self.map.width() * self.image_scale), int(self.map.height() * self.image_scale), Qt.KeepAspectRatio)
        
        # self.pixmap = self.pixmap.copy(QRect(50, 50, self.pixmap.width() - 30, self.pixmap.height() - 30))
        self.height = self.pixmap.size().height()
        self.width = self.pixmap.size().width()

        self.map_resolution = self.map_yaml_data['resolution']
        self.map_origin = self.map_yaml_data['origin'][:2]
        self.update_map()

    
    def init_main_page(self):
        # Main Page: Real-time location of robots, Task list, Current Stock info
        self.map_label = self.findChild(QLabel, 'map')  # Assuming there's a QLabel for the map


    def update_map(self):
        self.scaled_pixmap = self.pixmap.scaled(int(self.map.width() * self.image_scale), int(self.map.height() * self.image_scale), Qt.KeepAspectRatio)
        painter = QPainter(self.scaled_pixmap)

        # 로봇 번호 표시
        self.font = QFont()
        self.font.setBold(True)
        self.font.setPointSize(13)
        painter.setFont(self.font)

        # 1번 로봇 좌표
        self.draw_robot(painter, amcl_1, Qt.red, '1')

        # 2번 로봇 좌표
        # self.draw_robot(painter, amcl_2, Qt.blue, '2')
        
        # # 3번 로봇 좌표
        # self.draw_robot(painter, amcl_3, Qt.green, '3')
        painter.end()

        self.map.setPixmap(self.scaled_pixmap)

    def draw_robot(self, painter, amcl, color, label):
        x, y = self.calc_grid_position(amcl.pose.pose.position.x, amcl.pose.pose.position.y)
        # x, y = self.calc_grid_position(0.0, 0.0) # test용
        painter.setPen(QPen(color, 13, Qt.SolidLine))
        painter.drawPoint(int((self.width - x) * self.image_scale), int(y * self.image_scale))
        painter.drawText(int((self.width - x) * self.image_scale - 30), int(y * self.image_scale + 5), label)
    

    def calc_grid_position(self, x, y):
        x_offset = -85
        y_offset = 85
        x_grid = x_offset + ((x * 3.2 - self.map_origin[0]) / 0.05 )
        y_grid = y_offset + ((y * 3.0 - self.map_origin[1]) / 0.05 )
        return x_grid, y_grid
    
    def load_yaml(self, file_path):
        with open(file_path, 'r') as f:
            return yaml.full_load(f)


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    app = QApplication(sys.argv)

    window = Ui_MainWindow()
    window.show()

    amcl_node = AmclSubscriber()

    executor.add_node(amcl_node)

    thread = Thread(target=executor.spin)
    thread.start()

    app.exec_()

if __name__ == '__main__':
    main()
