import sys
sys.path.append('./db/src')  # DatabaseManager.py 파일의 경로를 추가

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5 import uic
from DatabaseManager import DatabaseManager
from barcode_scanner import BarcodeScanner

# manger_window = uic.loadUiType("gui/manager/ui/manager.ui")[0]

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
       
    def switch_page(self, page_index):
        self.stackedWidget.setCurrentIndex(page_index)
        
    def init_main_page(self):
        # Main Page: Real-time location of robots, Task list, Current Stock info
        self.MainPage = QWidget()
        self.stackedWidget.addWidget(self.MainPage)

        layout = QVBoxLayout(self.MainPage)
        self.location_label = QLabel("Real-time Location of Robots", self.MainPage)
        layout.addWidget(self.location_label)

        self.task_list = QTextEdit(self.MainPage)
        layout.addWidget(self.task_list)

        self.stock_info = QTableWidget(self.MainPage)
        self.stock_info.setColumnCount(3)
        self.stock_info.setHorizontalHeaderLabels(["Item Name", "Item ID", "Stock"])
        layout.addWidget(self.stock_info)

        self.update_stock_info()

    def update_stock_info(self):
        pass

    def init_robot_control_page(self):
        # Robot Control Page: Pi Cam, Status bar for each robot
        self.RobotControlPage = QWidget()
        self.stackedWidget.addWidget(self.RobotControlPage)

        layout = QVBoxLayout(self.RobotControlPage)
        self.robot_selector = QComboBox(self.RobotControlPage)
        self.robot_selector.addItems(["Robot_1", "Robot_2", "Robot_3"])
        self.robot_selector.currentIndexChanged.connect(self.update_robot_info)
        layout.addWidget(self.robot_selector)

        self.pi_cam_display = QLabel("Pi Cam Display", self.RobotControlPage)
        layout.addWidget(self.pi_cam_display)

        self.status_bar = QLabel("Status Bar", self.RobotControlPage)
        layout.addWidget(self.status_bar)

        self.update_robot_info()

    def update_robot_info(self):
        pass

    def init_inbound_order_control_page(self):
        # Inbound Order Control Page: Inbound list, Order list
        self.InboundOrderPage = QWidget()
        self.stackedWidget.addWidget(self.InboundOrderPage)

        layout = QVBoxLayout(self.InboundOrderPage)
        self.inbound_list = QTableWidget(self.InboundOrderPage)
        self.inbound_list.setColumnCount(5)
        self.inbound_list.setHorizontalHeaderLabels(["Item Name", "Quantity", "Inbound Zone", "Arrival Date", "Current Status"])
        layout.addWidget(self.inbound_list)

        self.order_list = QTableWidget(self.InboundOrderPage)
        self.order_list.setColumnCount(5)
        self.order_list.setHorizontalHeaderLabels(["Order ID", "Item Name", "Quantity", "Order Date", "Status"])
        layout.addWidget(self.order_list)

        self.scan_button = QPushButton("Scan Barcode", self.InboundOrderPage)
        self.scan_button.clicked.connect(self.scan_barcode)
        layout.addWidget(self.scan_button)

        self.barcode_scanner = BarcodeScanner()
        self.update_inbound_order_lists()

    def update_inbound_order_lists(self):
        pass

    def scan_barcode(self):
        pass

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Ui_MainWindow()
    window.show()
    sys.exit(app.exec_())
