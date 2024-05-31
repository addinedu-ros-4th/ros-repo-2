# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'testtnJOQL.ui'
##
## Created by: Qt User Interface Compiler version 5.15.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################
import sys
sys.path.append('./db/src')   # DatabaseManager.py 파일의 경로를 추가

import os
import json
from configparser import ConfigParser
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5 import uic

from datetime import datetime
from DatabaseManager import DatabaseManager
from websocket import create_connection
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.figure import Figure  # Figure 클래스를 명시적으로 가져옴
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import koreanize_matplotlib

from_class, _ = uic.loadUiType("gui/consumer/ui/main.ui")

class Ui_MainWindow(QMainWindow, from_class):
    def __init__(self, db_manager):
        super().__init__()
        self.db_manager = db_manager 
        self.setupUi(self)
        self.setWindowTitle("Main")
        
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        # 버튼 클릭 이벤트 연결
        self.home.clicked.connect(self.go_to_page1)
        self.order.clicked.connect(self.go_to_page2)
        self.go_order.clicked.connect(self.go_to_page2)
        self.chart.clicked.connect(self.go_to_page3)
        self.user.clicked.connect(self.go_to_page4)
        
        self.num_value = 0  # 숫자 값을 저장하는 변수
        self.user_id = self.db_manager.get_last_user_id() + 1  # 마지막 user_id에서 이어서 시작
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
            "item_name": [],
            "quantities": [],
            "timestamp": current_time
        }

        for row in range(self.model.rowCount()):
            item = self.model.item(row).text()
            product_name, quantity = item.split(": ")
            new_order["item_name"].append(product_name)
            new_order["quantities"].append(int(quantity))

        self.orders.append(new_order)
        
        # orders 리스트의 마지막 주문만 데이터베이스에 저장
        last_order = self.orders[-1]

        for item, quantity in zip(last_order["item_name"], last_order["quantities"]):
            product_id = self.db_manager.get_product_id(item)
            # print(f"save_to_database: item={item}, product_id={product_id}")  # 디버깅 정보 출력
            if product_id is None:
                QMessageBox.warning(self, "Error", f"Product ID for item '{item}' not found.")
                return
            stock = self.db_manager.get_stock(product_id)
            if stock is None:
                QMessageBox.warning(self, "Error", f"Stock for product ID {product_id} not found.")
                return
            if stock < quantity:
                QMessageBox.warning(self, "Stock Error", f"{item}은(는) 품절입니다.\n재고: {stock}")
                return

            data = {
                "user_id": last_order["user_id"],
                "order_time": last_order["timestamp"],
                "item_id": product_id,
                "item_name": item,
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
        # WebSocket을 통해 rosbridge로 연결
            ws = create_connection("ws://192.168.0.85:9090")
            # JSON 메시지 생성
            order_message = json.dumps({
                "op": "publish",
                "topic": "/order",
                "msg": {"data": json.dumps(self.orders)}
            })
            # 메시지 전송
            ws.send(order_message)
            ws.close()
        except OSError as e:
            QMessageBox.warning(self, "WebSocket Error", f"Failed to connect to WebSocket: {str(e)}")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"An error occurred: {str(e)}")
        
        
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(798, 600)
        MainWindow.setStyleSheet(u"background-color: rgb(255, 255, 255);")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        
        self.Wmenu = QWidget(self.centralwidget)
        self.Wmenu.setObjectName(u"Wmenu")
        self.Wmenu.setGeometry(QRect(0, 0, 81, 601))
        self.Wmenu.setStyleSheet(u"background-color: rgb(212,212,212);")
        
        self.home = QPushButton(self.Wmenu)
        self.home.setObjectName(u"home")
        self.home.setGeometry(QRect(10, 100, 61, 61))
        self.home.setStyleSheet(u"background-color: rgb(255, 255, 255);\n""border-radius: 30px")
        icon = QIcon()
        icon.addFile(u"gui/consumer/image/home.png", QSize(), QIcon.Normal, QIcon.Off)
        self.home.setIcon(icon)
        self.home.setIconSize(QSize(25, 25))
        
        self.order = QPushButton(self.Wmenu)
        self.order.setObjectName(u"order")
        self.order.setGeometry(QRect(10, 220, 61, 61))
        self.order.setStyleSheet(u"background-color: rgb(255, 255, 255);\n""border-radius: 30px")
        icon1 = QIcon()
        icon1.addFile(u"gui/consumer/image/order.png", QSize(), QIcon.Normal, QIcon.Off)
        self.order.setIcon(icon1)
        self.order.setIconSize(QSize(30, 30))
        
        self.chart = QPushButton(self.Wmenu)
        self.chart.setObjectName(u"chart")
        self.chart.setGeometry(QRect(10, 350, 61, 61))
        self.chart.setStyleSheet(u"background-color: rgb(255, 255, 255);\n""border-radius: 30px")
        icon2 = QIcon()
        icon2.addFile(u"gui/consumer/image/bar_chart.png", QSize(), QIcon.Normal, QIcon.Off)
        self.chart.setIcon(icon2)
        self.chart.setIconSize(QSize(30, 30))
        
        self.label = QLabel(self.Wmenu)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(20, 20, 41, 17))
        
        self.user = QPushButton(self.Wmenu)
        self.user.setObjectName(u"user")
        self.user.setGeometry(QRect(10, 480, 61, 61))
        self.user.setStyleSheet(u"background-color: rgb(255, 255, 255);\n""border-radius: 30px")
        icon3 = QIcon()
        icon3.addFile(u"gui/consumer/image/user.png", QSize(), QIcon.Normal, QIcon.Off)
        self.user.setIcon(icon3)
        self.user.setIconSize(QSize(30, 30))
        
        self.stackedWidget = QStackedWidget(self.centralwidget)
        self.stackedWidget.setObjectName(u"stackedWidget")
        self.stackedWidget.setGeometry(QRect(80, 0, 721, 611))
        self.stackedWidget.setMinimumSize(QSize(10, 10))
        self.stackedWidget.setStyleSheet(u"background-color: rgb(255, 255, 255);")
        
        self.page = QWidget()
        self.page.setObjectName(u"page")
        
        self.widget = QWidget(self.page)
        self.widget.setObjectName(u"widget")
        self.widget.setGeometry(QRect(0, 150, 721, 281))
        self.widget.setStyleSheet(u"background-color: rgb(82, 98, 116);")
        
        self.go_order = QPushButton(self.widget)
        self.go_order.setObjectName(u"go_order")
        self.go_order.setGeometry(QRect(270, 110, 121, 61))
        font = QFont()
        font.setBold(True)
        font.setWeight(75)
        self.go_order.setFont(font)
        self.go_order.setStyleSheet(u"background-color: rgb(255, 255, 255);\n""border-radius: 10px")
        
        self.stackedWidget.addWidget(self.page)
        
        self.page_2 = QWidget()
        self.page_2.setObjectName(u"page_2")
        self.page_2.setStyleSheet(u"background-color: rgb(82, 98, 116);")
        
        self.minus = QPushButton(self.page_2)
        self.minus.setObjectName(u"minus")
        self.minus.setGeometry(QRect(350, 180, 41, 25))
        font1 = QFont()
        font1.setPointSize(17)
        font1.setBold(True)
        font1.setWeight(75)
        self.minus.setFont(font1)
        self.minus.setStyleSheet(u"	border:none;\n")
        
        self.plus = QPushButton(self.page_2)
        self.plus.setObjectName(u"plus")
        self.plus.setGeometry(QRect(470, 180, 41, 25))
        self.plus.setFont(font1)
        self.plus.setStyleSheet(u"border:none;\n")
        
        self.buy_btn = QPushButton(self.page_2)
        self.buy_btn.setObjectName(u"buy_btn")
        self.buy_btn.setGeometry(QRect(480, 410, 89, 51))
        self.buy_btn.setStyleSheet(u"color:#000;\n""border:none;\n""")
        icon4 = QIcon()
        icon4.addFile(u"gui/consumer/image/buy.png", QSize(), QIcon.Normal, QIcon.Off)
        self.buy_btn.setIcon(icon4)
        self.buy_btn.setIconSize(QSize(80, 80))
        
        self.add_btn = QPushButton(self.page_2)
        self.add_btn.setObjectName(u"add_btn")
        self.add_btn.setGeometry(QRect(530, 180, 41, 31))
        self.add_btn.setStyleSheet(u"border: none;")
        icon6 = QIcon()
        icon6.addFile(u"gui/consumer/image/cart.png", QSize(), QIcon.Normal, QIcon.Off)
        self.add_btn.setIcon(icon6)
        self.add_btn.setIconSize(QSize(35, 35))
        
        self.select = QComboBox(self.page_2)
        self.select.setObjectName(u"select")
        self.select.setGeometry(QRect(130, 134, 441, 31))
        self.select.setStyleSheet(u"background-color: rgb(255, 255, 255);\n""\n""")
        
        self.num = QLineEdit(self.page_2)
        self.num.setObjectName(u"num")
        self.num.setGeometry(QRect(400, 180, 61, 25))
        
        self.listView = QListView(self.page_2)
        self.listView.setObjectName(u"listView")
        self.listView.setGeometry(QRect(130, 230, 441, 171))
        self.listView.setStyleSheet(u"background-color: rgb(255, 255, 255);")
        
        self.delete_btn = QPushButton(self.page_2)
        self.delete_btn.setObjectName(u"delete_btn")
        self.delete_btn.setGeometry(QRect(130, 420, 41, 25))
        self.delete_btn.setStyleSheet(u"border:none;")
        icon6 = QIcon()
        icon6.addFile(u"gui/consumer/image/delete.png", QSize(), QIcon.Normal, QIcon.Off)
        self.delete_btn.setIcon(icon6)
        self.delete_btn.setIconSize(QSize(25, 26))
        
        self.stackedWidget.addWidget(self.page_2)
        
        self.page_3 = QWidget()
        self.page_3.setObjectName(u"page_3")
        self.page_3.setStyleSheet(u"background-color: rgb(255, 255, 255);")
        
        self.stats = QLabel(self.page_3)
        self.stats.setObjectName(u"stats")
        self.stats.setGeometry(QRect(40, 30, 67, 17))
        
        self.widget1 = QWidget(self.page_3)
        self.widget1.setObjectName(u"widget1")
        self.widget1.setGeometry(QRect(30, 30, 651, 541))
        
        self.verticalLayout = QVBoxLayout(self.widget1)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        
        self.circle_chart = QWidget(self.widget1)
        self.circle_chart.setObjectName(u"circle_chart")
        self.circle_chart.setLayout(QVBoxLayout())  # Explicitly setting a layout

        self.verticalLayout.addWidget(self.circle_chart)

        self.clock = QLabel(self.widget1)
        self.clock.setObjectName(u"clock")

        self.verticalLayout.addWidget(self.clock)

        self.time_chart = QWidget(self.widget1)
        self.time_chart.setObjectName(u"time_chart")
        self.time_chart.setLayout(QVBoxLayout())  # Explicitly setting a layout

        self.verticalLayout.addWidget(self.time_chart)

        self.stackedWidget.addWidget(self.page_3)
        
        self.page_4 = QWidget()
        self.page_4.setObjectName(u"page_4")
        self.page_4.setStyleSheet(u"background-color: rgb(82, 98, 116);")
        
        self.userinfo = QLabel(self.page_4)
        self.userinfo.setObjectName(u"userinfo")
        self.userinfo.setGeometry(QRect(580, 50, 67, 17))
        
        self.usericon = QPushButton(self.page_4)
        self.usericon.setObjectName(u"pushButton")
        self.usericon.setGeometry(QRect(530, 40, 41, 41))
        self.usericon.setStyleSheet(u"border: none;")
        icon7 = QIcon()
        icon7.addFile(u"gui/consumer/image/user2.png", QSize(), QIcon.Normal, QIcon.Off)
        self.usericon.setIcon(icon7)
        self.usericon.setIconSize(QSize(30, 30))
        
        self.orderinfo = QLabel(self.page_4)
        self.orderinfo.setObjectName(u"orderinfo")
        self.orderinfo.setGeometry(QRect(60, 90, 67, 17))
        self.tableWidget = QTableWidget(self.page_4)
        
        if (self.tableWidget.columnCount() < 5):
            self.tableWidget.setColumnCount(5)
            
        __qtablewidgetitem = QTableWidgetItem()
        self.tableWidget.setHorizontalHeaderItem(0, __qtablewidgetitem)
        __qtablewidgetitem1 = QTableWidgetItem()
        self.tableWidget.setHorizontalHeaderItem(1, __qtablewidgetitem1)
        __qtablewidgetitem2 = QTableWidgetItem()
        self.tableWidget.setHorizontalHeaderItem(2, __qtablewidgetitem2)
        __qtablewidgetitem3 = QTableWidgetItem()
        self.tableWidget.setHorizontalHeaderItem(3, __qtablewidgetitem3)
        __qtablewidgetitem4 = QTableWidgetItem()
        
        self.tableWidget.setHorizontalHeaderItem(4, __qtablewidgetitem4)
        self.tableWidget.setObjectName(u"tableWidget")
        self.tableWidget.setGeometry(QRect(60, 130, 601, 192))
        
        self.stackedWidget.addWidget(self.page_4)
        
        MainWindow.setCentralWidget(self.centralwidget)
        self.stackedWidget.raise_()
        self.Wmenu.raise_()
        self.retranslateUi(MainWindow)
        self.stackedWidget.setCurrentIndex(0)
        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.home.setText("")
        self.order.setText("")
        self.chart.setText("")
        self.label.setText(QCoreApplication.translate("MainWindow", u"menu", None))
        self.user.setText("")
        self.go_order.setText(QCoreApplication.translate("MainWindow", u"주문하기", None))
        self.minus.setText(QCoreApplication.translate("MainWindow", u"-", None))
        self.plus.setText(QCoreApplication.translate("MainWindow", u"+", None))
        self.buy_btn.setText("")
        self.select.addItem(QCoreApplication.translate("OrderWindow", u"선택해주세요", None))
        self.select.addItem(QCoreApplication.translate("OrderWindow", u"cola", None))
        self.select.addItem(QCoreApplication.translate("OrderWindow", u"cider", None))
        self.select.addItem(QCoreApplication.translate("OrderWindow", u"coffee", None))
        self.select.addItem(QCoreApplication.translate("OrderWindow", u"water", None))
        self.select.addItem(QCoreApplication.translate("OrderWindow", u"Jin_ramen", None))
        self.select.addItem(QCoreApplication.translate("OrderWindow", u"Chapagetti", None))
        self.select.addItem(QCoreApplication.translate("OrderWindow", u"Bibimmyeon", None))
        self.select.addItem(QCoreApplication.translate("OrderWindow", u"robot_cleaner", None))
        self.select.addItem(QCoreApplication.translate("OrderWindow", u"radio", None))
        self.select.addItem(QCoreApplication.translate("OrderWindow", u"tv", None))
        self.add_btn.setText("")

        self.delete_btn.setText("")
        self.stats.setText(QCoreApplication.translate("MainWindow", u"", None))
        self.userinfo.setText(QCoreApplication.translate("MainWindow", u"user id : 2", None))
        self.usericon.setText("")
        self.orderinfo.setText(QCoreApplication.translate("MainWindow", u"주문정보", None))
        ___qtablewidgetitem = self.tableWidget.horizontalHeaderItem(0)
        ___qtablewidgetitem.setText(QCoreApplication.translate("MainWindow", u"주문번호", None));
        ___qtablewidgetitem1 = self.tableWidget.horizontalHeaderItem(1)
        ___qtablewidgetitem1.setText(QCoreApplication.translate("MainWindow", u"상품번호", None));
        ___qtablewidgetitem2 = self.tableWidget.horizontalHeaderItem(2)
        ___qtablewidgetitem2.setText(QCoreApplication.translate("MainWindow", u"상품명", None));
        ___qtablewidgetitem3 = self.tableWidget.horizontalHeaderItem(3)
        ___qtablewidgetitem3.setText(QCoreApplication.translate("MainWindow", u"갯수", None));
        ___qtablewidgetitem4 = self.tableWidget.horizontalHeaderItem(4)
        ___qtablewidgetitem4.setText(QCoreApplication.translate("MainWindow", u"주문시각", None));
    # retranslateUi
    
    def go_to_page1(self):
        self.stackedWidget.setCurrentIndex(0)
        
    def go_to_page2(self):
        self.stackedWidget.setCurrentIndex(1)
    
    def go_to_page3(self):
        self.stackedWidget.setCurrentIndex(2)
        self.update_page3()

    def go_to_page4(self):
        self.stackedWidget.setCurrentIndex(3)

    def update_page3(self):
        product_orders = self.db_manager.fetch_all_product_orders()
        df = pd.DataFrame(product_orders, columns=['OrderID', 'UserID', 'ItemID', 'ProductName', 'Quantity', 'OrderTime'])  # 열 순서 수정
        # print(df)  # 데이터가 제대로 들어왔는지 확인
        self.plot_pie_chart(df)
        self.display_most_ordered_hour(df)
        self.plot_time_chart(df)

    def plot_pie_chart(self, df):
        product_quantity = df.groupby('ProductName')['Quantity'].sum()
        colors = plt.get_cmap('tab20').colors 
        figure = Figure(figsize=(6, 6))
        ax = figure.add_subplot(111)
        product_quantity.plot.pie(ax=ax, autopct='%1.1f%%', startangle=100,
                                textprops={'fontsize': 7}, colors=colors) 
        ax.set_ylabel('')

        layout = self.circle_chart.layout()
        for i in reversed(range(layout.count())):
            layout.itemAt(i).widget().setParent(None)
        
        title_label = QLabel("상품별 주문량")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("font-size: 15px;")
        layout.addWidget(title_label)

        canvas = FigureCanvas(figure)
        layout.addWidget(canvas)

    def plot_time_chart(self, df):
        df['OrderTime'] = pd.to_datetime(df['OrderTime'])
        df['Hour'] = df['OrderTime'].dt.hour
        hourly_order_sum = df.groupby('Hour')['Quantity'].sum()

        figure = Figure(figsize=(8, 5))
        ax = figure.add_subplot(111)
        hourly_order_sum.plot(ax=ax, kind='line', marker='o', color='green')  # 라인 차트 색상 변경
        # ax.set_title('Hourly Orders Quantity', fontsize=10)  # 제목의 글자 크기 조정
        # ax.set_xlabel('Hour of the Day', fontsize=8)  # x축 레이블의 글자 크기 조정
        # ax.set_ylabel('Total Quantity Ordered', fontsize=8)  # y축 레이블의 글자 크기 조정
        ax.tick_params(axis='both', which='major', labelsize=7)  # 축 레이블의 글자 크기 조정

        canvas = FigureCanvas(figure)
        layout = self.time_chart.layout()
        for i in reversed(range(layout.count())):
            layout.itemAt(i).widget().setParent(None)
        layout.addWidget(canvas)

    def display_most_ordered_hour(self, df):
        df['OrderTime'] = pd.to_datetime(df['OrderTime'])
        df['Hour'] = df['OrderTime'].dt.hour
        most_ordered_hour = df['Hour'].value_counts().idxmax()
        self.clock.setText(f"주문 많이 한 시간: {most_ordered_hour}:00")
        self.clock.setAlignment(Qt.AlignCenter)

    def update_page4(self):
        # userinfo 업데이트
        self.userinfo.setText(f"user id : {self.user_id}")

        # 제품 주문 데이터 가져오기
        product_orders = self.db_manager.fetch_all_product_orders()

        # DataFrame으로 변환
        df = pd.DataFrame(product_orders, columns=['OrderID', 'UserID', 'ItemID', 'ProductName', 'Quantity', 'OrderTime'])  # 열 순서 수정
        df = df.drop(columns=['UserID'])  # user_id 칼럼 제외
        # print(df)  # 데이터가 제대로 들어왔는지 확인

        # tableWidget 업데이트
        self.tableWidget.setRowCount(len(df))
        self.tableWidget.setColumnCount(len(df.columns))
        self.tableWidget.setHorizontalHeaderLabels(df.columns)

        for row in df.itertuples():
            for col_index, value in enumerate(row[1:]):
                item = QTableWidgetItem(str(value))
                self.tableWidget.setItem(row.Index, col_index, item)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # config 파일에서 host 정보를 읽어옴
    config_path = 'db/config/config.ini'
    config = ConfigParser()
    config.read(config_path)
    
    # 디버깅: 설정 파일의 내용을 출력하여 확인
    print("Configuration file sections:", config.sections())
    
    if not config.has_section('database'):
        print("Error: 'database' section not found in the configuration file")
        sys.exit(1)
    
    host = config.get('database', 'host')
    
    try:
        db_manager = DatabaseManager(host)
        db_manager.connect_database()
        db_manager.create_table()
    except Exception as e:
        print(f"Failed to initialize the database manager: {e}")
        sys.exit(1)

    window = Ui_MainWindow(db_manager)
    window.show()
    sys.exit(app.exec_())
