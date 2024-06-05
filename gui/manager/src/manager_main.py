import sys
sys.path.append('./db/src')  # DatabaseManager.py 파일의 경로를 추가

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5 import uic
from DatabaseManager import DatabaseManager
from barcode_scanner import BarcodeScanner
from RobotController import RobotController
import pandas as pd

class Ui_MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Manager Window")

        self.db_manager = DatabaseManager(host='localhost')
        self.db_manager.connect_database()
        self.db_manager.create_table()
        
        self.robotstatus = RobotController(host='localhost')

        # Load UI
        uic.loadUi("gui/manager/ui/manager.ui", self)

        # Initialize the Stacked Widget
        self.stackedWidget = self.findChild(QStackedWidget, 'stackedWidget')

        # Initialize pages
        self.init_main_page()
        self.init_robot_control_page()
        self.init_inbound_order_control_page()

        self.init_navigation_buttons()

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
        # Initialize main page elements
        self.update_stock_info()

        robotstatus = self.db_manager.utils.fetch_all_product("RobotStatus")

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
                
    def update_stock_info(self):
        print("Updating stock info")
        product_inventory = self.db_manager.utils.fetch_all_product("ProductInventory")
        print(f"Fetched product inventory: {product_inventory}")

        df = pd.DataFrame(product_inventory, columns=['item_id', 'item_name', 'stock'])

        # tableWidget 업데이트
        self.tableWidget.setRowCount(len(df))
        self.tableWidget.setColumnCount(len(df.columns))
        self.tableWidget.setHorizontalHeaderLabels(df.columns)

        for row_index, row in enumerate(df.itertuples(index=False)):
            for col_index, value in enumerate(row):
                item = QTableWidgetItem(str(value))
                self.tableWidget.setItem(row_index, col_index, item)
        print("Stock info updated")
        
    def update_product_quantity(self, product_id, quantity):
        query = "UPDATE ProductInventory SET stock = %s WHERE item_id = %s"
        self.db_manager.cursor.execute(query, (quantity, product_id))
        self.db_manager.conn.commit()
        print(f"Updated product {product_id} with quantity {quantity}")

    def update_map(self):
        pass
    
    def update_task_list(self):
        pass

    def init_robot_control_page(self):
        self.robotComboBox = self.findChild(QComboBox, 'robotComboBox')
        self.picamLabel = self.findChild(QLabel, 'picamLabel')
        self.statusLabel = self.findChild(QLabel, 'statusLabel')

        if self.robotComboBox is None:
            print("Error: robotComboBox not found")
        else:
            self.robotComboBox.addItems(["robot_1", "robot_2", "robot_3"])
        self.robotComboBox.currentIndexChanged.connect(self.update_robot_info)

        # Initialize with the first robot's data
        #self.update_robot_info(0)

    def update_robot_info(self):
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

        # Get all rows from the Inbound table
        all_rows = self.db_manager.get_data("Inbound", ["item_name", "quantity", "inbound_zone", "arrival_date", "current_status"])
        
        print("Inbound table data:", all_rows)  # Debugging print statement

        # Populate the QTableWidget with data from the Inbound table
        for row in all_rows:
            row_position = self.inbound_list.rowCount()
            self.inbound_list.insertRow(row_position)
            for column, value in enumerate(row):
                self.inbound_list.setItem(row_position, column, QTableWidgetItem(str(value)))
        print("Updated inbound_list with new data")  # Debugging print statement

    def scan_barcode(self):
        import threading
        threading.Thread(target=self.barcode_scanner.append_list).start()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Ui_MainWindow()
    window.show()
    sys.exit(app.exec_())
