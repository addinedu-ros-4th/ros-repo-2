import sys
sys.path.append('./db/src')  # DatabaseManager.py 파일의 경로를 추가

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5 import uic
from DatabaseManager import DatabaseManager
from barcode_scanner import BarcodeScanner

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
        pass

    def update_stock_info(self):
        pass

    def init_robot_control_page(self):
        pass

    def update_robot_info(self):
        pass

    def init_inbound_order_control_page(self):
        self.scanned_data = ""
        self.scan_button.clicked.connect(self.scan_barcode)


        self.barcode_scanner = BarcodeScanner()  # Properly initialize BarcodeScanner
        self.barcode_scanner.barcode_scanned.connect(self.update_inbound_list)  # Connect signal to slot

    def update_inbound_list(self, data=None):
        # Clear the existing rows in the QTableWidget
        self.inbound_list.setRowCount(0)

        # Get all rows from the Inbound table
        all_rows = self.db_manager.get_data("Inbound")
        
        print("Inbound table data:", all_rows)  # Debugging print statement

        # Populate the QTableWidget with data from the Inbound table
        for row in all_rows:
            row_position = self.inbound_list.rowCount()
            self.inbound_list.insertRow(row_position)
            for column, value in enumerate(row):
                self.inbound_list.setItem(row_position, column, QTableWidgetItem(str(value)))
        print("Updated inbound_list with new data")  # Debugging print statement

    def scan_barcode(self):
        self.barcode_scanner.append_list()
        self.update_inbound_list()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Ui_MainWindow()
    window.show()
    sys.exit(app.exec_())
