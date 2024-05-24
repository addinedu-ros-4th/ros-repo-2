import sys
sys.path.append('/home/addinedu/testdb')  # DatabaseManager.py 파일의 경로를 추가

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5 import uic
from datetime import datetime
from DatabaseManager import DatabaseManager
from websocket import create_connection

from_orderpage_class = uic.loadUiType("gui/ui/order.ui")[0]

class Ui_OrderWindow(QMainWindow, from_orderpage_class):
    def __init__(self, db_manager):
        super().__init__()
        self.db_manager = db_manager  # db_manager 객체를 인스턴스 변수로 저장
        self.setupUi(self)
        self.setWindowTitle("Order Page")

        self.num_value = 0  # 숫자 값을 저장하는 변수
        self.user_id = 0  # 유저 아이디를 저장하는 변수, 초기값 0
        self.num.setText(str(self.num_value))  # 초기값 설정
        self.num.setReadOnly(True)  # QLineEdit을 읽기 전용으로 설정
        
        self.orders = []  # 모든 주문을 저장할 리스트

        self.plus.clicked.connect(self.increase_num)
        self.minus.clicked.connect(self.decrease_num)
        self.add_btn.clicked.connect(self.add_to_list)
        self.buy_btn.clicked.connect(self.save_to_database)
        self.delete_btn.clicked.connect(self.delete_from_list)
        
        self.model = QStandardItemModel(self.listView)  # QStandardItemModel 생성
        self.listView.setModel(self.model)  # QListView에 모델 설정

    def increase_num(self):
        self.num_value += 1
        self.num.setText(str(self.num_value))
        
    def decrease_num(self):
        if self.num_value > 0:
            self.num_value -= 1
        self.num.setText(str(self.num_value))

    def add_to_list(self):
        product_name = self.select.currentText()  # 선택된 상품명 가져오기
        quantity = self.num.text()  # 수량 가져오기
        list_item = QStandardItem(f"{product_name}: {quantity}")
        self.model.appendRow(list_item)  # 모델에 항목 추가

    def delete_from_list(self):
        selected_index = self.listView.selectedIndexes()
        if selected_index:
            index = selected_index[0]
            self.model.removeRow(index.row())

    def save_to_database(self):
        # 현재 시각을 가져오기
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        # 마지막 주문을 orders 리스트에 추가
        new_order = {
            "user_id": self.user_id,
            "items": [],
            "quantities": [],
            "timestamp": current_time
        }

        for row in range(self.model.rowCount()):
            item = self.model.item(row).text()
            product_name, quantity = item.split(": ")
            new_order["items"].append(product_name)
            new_order["quantities"].append(int(quantity))

        self.orders.append(new_order)
        
        # orders 리스트의 마지막 주문만 데이터베이스에 저장
        last_order = self.orders[-1]

        for item, quantity in zip(last_order["items"], last_order["quantities"]):
            product_id = self.db_manager.get_product_id(item)
            stock = self.db_manager.get_stock(product_id)
            if stock is None:
                QMessageBox.warning(self, "Error", f"Product ID {product_id} not found.")
                return
            if stock < quantity:
                QMessageBox.warning(self, "Stock Error", f"{item}은(는) 품절입니다.\n재고: {stock}")
                return

            data = {
                "user_id": last_order["user_id"],
                "order_time": last_order["timestamp"],
                "item_id": product_id,
                "items": item,
                "quantities": quantity
            }
            self.db_manager.save_data("ProductOrder", data)
            self.db_manager.update_stock(product_id, quantity)

            # 재고 현황 프린트
            print(f"Product: {item}, Stock after order: {stock - quantity}")

        QMessageBox.information(self, "Saved", "결제완료")

        # user_id 증가
        self.user_id += 1

        # 리스트 뷰 초기화
        self.model.clear()
        self.num_value = 0
        self.num.setText(str(self.num_value))

        # 웹소켓 연결 시도
        self.send_task_to_ros()

    def send_task_to_ros(self):
        try:
            ws = create_connection("ws://192.168.0.85:9090")
            ws.send("Task message")  # 실제로 보내고자 하는 메시지로 수정
            ws.close()
        except OSError as e:
            QMessageBox.warning(self, "WebSocket Error", f"Failed to connect to WebSocket: {str(e)}")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"An error occurred: {str(e)}")

    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(800, 600)
        MainWindow.setStyleSheet(u"background-color: rgb(82, 98, 116);")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        
        self.select = QComboBox(self.centralwidget)
        self.select.addItem("")
        self.select.addItem("")
        self.select.addItem("")
        self.select.setObjectName(u"select")
        self.select.setGeometry(QRect(210, 134, 441, 31))
        self.select.setStyleSheet(u"background-color: rgb(255, 255, 255);\n""\n""")
        self.select.clear()  # 기존 항목 제거
        
        self.num = QLineEdit(self.centralwidget)
        self.num.setObjectName(u"num")
        self.num.setGeometry(QRect(480, 180, 61, 25))
        self.num.setStyleSheet(u"background-color: rgb(255, 255, 255);")
        self.num.setAlignment(Qt.AlignCenter)
        
        self.minus = QPushButton(self.centralwidget)
        self.minus.setObjectName(u"minus")
        self.minus.setGeometry(QRect(430, 180, 41, 25))
        font = QFont()
        font.setPointSize(17)
        font.setBold(True)
        font.setWeight(75)
        self.minus.setFont(font)
        self.minus.setStyleSheet(u"\n""	border:none;\n""")
        
        self.plus = QPushButton(self.centralwidget)
        self.plus.setObjectName(u"plus")
        self.plus.setGeometry(QRect(550, 180, 41, 25))
        self.plus.setFont(font)
        self.plus.setStyleSheet(u"\n""	border:none;\n""")
        
        self.buy_btn = QPushButton(self.centralwidget)
        self.buy_btn.setObjectName(u"buy_btn")
        self.buy_btn.setGeometry(QRect(560, 410, 89, 51))
        self.buy_btn.setStyleSheet(u"\n""	color:#000;\n""	border:none;\n""")
        icon = QIcon()
        icon.addFile(u"gui/image/buy.png", QSize(), QIcon.Normal, QIcon.Off)
        self.buy_btn.setIcon(icon)
        self.buy_btn.setIconSize(QSize(80, 80))
        
        self.listView = QListView(self.centralwidget)
        self.listView.setObjectName(u"listView")
        self.listView.setGeometry(QRect(210, 230, 441, 171))
        self.listView.setStyleSheet(u"background-color: rgb(255, 255, 255);")
        
        self.Wmenu = QWidget(self.centralwidget)
        self.Wmenu.setObjectName(u"Wmenu")
        self.Wmenu.setGeometry(QRect(0, 0, 81, 601))
        self.Wmenu.setStyleSheet(u"background-color: rgb(212,212,212);")
        
        self.home = QPushButton(self.Wmenu)
        self.home.setObjectName(u"home")
        self.home.setGeometry(QRect(10, 100, 61, 61))
        self.home.setStyleSheet(u"background-color: rgb(255, 255, 255);\n""border-radius: 30px")
        icon1 = QIcon()
        icon1.addFile(u"gui/image/home.png", QSize(), QIcon.Normal, QIcon.Off)
        self.home.setIcon(icon1)
        self.home.setIconSize(QSize(25, 25))
        
        self.order = QPushButton(self.Wmenu)
        self.order.setObjectName(u"order")
        self.order.setGeometry(QRect(10, 220, 61, 61))
        self.order.setStyleSheet(u"background-color: rgb(255, 255, 255);\n""border-radius: 30px")
        icon2 = QIcon()
        icon2.addFile(u"gui/image/order.png", QSize(), QIcon.Normal, QIcon.Off)
        self.order.setIcon(icon2)
        self.order.setIconSize(QSize(30, 30))
        
        self.chart = QPushButton(self.Wmenu)
        self.chart.setObjectName(u"chart")
        self.chart.setGeometry(QRect(10, 350, 61, 61))
        self.chart.setStyleSheet(u"background-color: rgb(255, 255, 255);\n""border-radius: 30px")
        icon3 = QIcon()
        icon3.addFile(u"gui/image/bar_chart.png", QSize(), QIcon.Normal, QIcon.Off)
        self.chart.setIcon(icon3)
        self.chart.setIconSize(QSize(30, 30))
        
        self.label = QLabel(self.Wmenu)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(20, 20, 41, 17))
        
        self.user = QPushButton(self.Wmenu)
        self.user.setObjectName(u"user")
        self.user.setGeometry(QRect(10, 480, 61, 61))
        self.user.setStyleSheet(u"background-color: rgb(255, 255, 255);\n""border-radius: 30px")
        icon4 = QIcon()
        icon4.addFile(u"gui/image/user.png", QSize(), QIcon.Normal, QIcon.Off)
        self.user.setIcon(icon4)
        self.user.setIconSize(QSize(30, 30))
        
        self.add_btn = QPushButton(self.centralwidget)
        self.add_btn.setObjectName(u"add_btn")
        self.add_btn.setGeometry(QRect(610, 180, 41, 31))
        self.add_btn.setStyleSheet(u"border:none;")
        
        icon5 = QIcon()
        icon5.addFile(u"gui/image/cart.png", QSize(), QIcon.Normal, QIcon.Off)
        self.add_btn.setIcon(icon5)
        self.add_btn.setIconSize(QSize(35, 35))
        
        self.delete_btn = QPushButton(self.centralwidget)
        self.delete_btn.setObjectName(u"delete_btn")
        self.delete_btn.setGeometry(QRect(210, 420, 41, 25))
        self.delete_btn.setStyleSheet(u"border:none;")
        icon6 = QIcon()
        icon6.addFile(u"gui/image/delete.png", QSize(), QIcon.Normal, QIcon.Off)
        self.delete_btn.setIcon(icon6)
        self.delete_btn.setIconSize(QSize(25, 26))
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi
    
    def retranslateUi(self, OrderWindow):
        OrderWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"Order Page", None))
        self.select.addItem(QCoreApplication.translate("OrderWindow", u"선택해주세요", None))
        self.select.addItem(QCoreApplication.translate("OrderWindow", u"cola", None))
        self.select.addItem(QCoreApplication.translate("OrderWindow", u"water", None))
        self.select.addItem(QCoreApplication.translate("OrderWindow", u"ramen", None))
        
        self.minus.setText(QCoreApplication.translate("MainWindow", u"-", None))
        self.plus.setText(QCoreApplication.translate("MainWindow", u"+", None))
        self.buy_btn.setText("")
        self.home.setText("")
        self.order.setText("")
        self.chart.setText("")
        self.label.setText(QCoreApplication.translate("MainWindow", u"menu", None))
        self.user.setText("")
        self.num.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.add_btn.setText("")
        
        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    db_manager = DatabaseManager(host='localhost')
    db_manager.connect_database()
    db_manager.create_table()
    order_window = Ui_OrderWindow(db_manager)
    order_window.show()
    sys.exit(app.exec_())
