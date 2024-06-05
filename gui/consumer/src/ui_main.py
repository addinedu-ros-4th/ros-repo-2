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
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import koreanize_matplotlib
from ui_setup import Ui_Setup  # ui_setup 모듈 임포트

# from_class, _ = uic.loadUiType("gui/consumer/ui/main.ui")

class Ui_MainWindow(QMainWindow, Ui_Setup):
    def __init__(self, db_manager):
        super().__init__()
        self.db_manager = db_manager 
        self.user_id = self.db_manager.utils.get_last_user_id("ProductOrder") + 1  # 마지막 user_id에서 이어서 시작
        self.setupUi(self)
        self.setWindowTitle("Main")
        
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        self.home.clicked.connect(self.go_to_home)
        self.order.clicked.connect(self.go_to_order)
        self.go_order.clicked.connect(self.go_to_order)
        self.chart.clicked.connect(self.go_to_chart)
        self.user.clicked.connect(self.go_to_user)
        
        self.num_value = 0  
        self.num.setText(str(self.num_value))  
        self.num.setReadOnly(True)  
        
        self.orders = []  # 모든 주문을 저장할 리스트

        self.plus.clicked.connect(self.increase_num)
        self.minus.clicked.connect(self.decrease_num)
        self.add_btn.clicked.connect(self.add_to_list)
        self.buy_btn.clicked.connect(self.save_to_database)
        self.delete_btn.clicked.connect(self.delete_from_list)
        
        self.model = QStandardItemModel(self.listView)  
        self.listView.setModel(self.model)
    
    
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
        out_of_stock = False
        
        for item, quantity in zip(last_order["item_name"], last_order["quantities"]):
            product_id = self.db_manager.utils.get_product_id(item)
            # print(f"save_to_database: item={item}, product_id={product_id}")  # 디버깅 정보 출력
            if product_id is None:
                QMessageBox.warning(self, "Error", f"Product ID for item '{item}' not found.")
                return
            stock = self.db_manager.utils.get_stock(product_id)
            if stock is None:
                QMessageBox.warning(self, "Error", f"Stock for product ID {product_id} not found.")
                return
            if stock < quantity:
                QMessageBox.warning(self, "Stock Error", f"{item}은(는) 품절입니다.\n재고: {stock}")
                out_of_stock = True
                break

            data = {
                "user_id": last_order["user_id"],
                "order_time": last_order["timestamp"],
                "item_id": product_id,
                "item_name": item,
                "quantities": quantity
            }
            self.db_manager.save_data("ProductOrder", data)
            self.db_manager.utils.update_stock(product_id, quantity)

            # 재고 현황 프린트
            print(f"Product: {item}, Stock after order: {stock - quantity}")
        
        if out_of_stock:
            return
        
        QMessageBox.information(self, "Saved", "결제완료")

        self.user_id += 1

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
                "topic": "/outbound",
                "msg": {"data": json.dumps(self.orders)}
            })
            # 메시지 전송
            ws.send(order_message)
            ws.close()
        except OSError as e:
            QMessageBox.warning(self, "WebSocket Error", f"Failed to connect to WebSocket: {str(e)}")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"An error occurred: {str(e)}")
        

    def go_to_home(self):
        self.stackedWidget.setCurrentIndex(0)
        
        
    def go_to_order(self):
        self.stackedWidget.setCurrentIndex(1)
    
    
    def go_to_chart(self):
        self.stackedWidget.setCurrentIndex(2)
        self.chartPage()


    def go_to_user(self):
        self.stackedWidget.setCurrentIndex(3)
        self.ordercheckPage()


    def chartPage(self):
        df = self.db_manager.utils.fetch_product_orders_dataframe()
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
        hourly_order_sum.plot(ax=ax, kind='line', marker='o', color='green')
        ax.tick_params(axis='both', which='major', labelsize=7)  

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

    # 주문조회 페이지
    def ordercheckPage(self):
        previous_user_id = self.user_id - 1
        self.userinfo.setText(f"user id : {previous_user_id}")

        product_orders = self.db_manager.utils.fetch_all_product("ProductOrder")

        df = pd.DataFrame(product_orders, columns=['주문번호', '사용자', '상품번호', '상품명', '갯수', '주문시각'])
        df = df[df['사용자'] == previous_user_id]
        df = df.drop(columns=['사용자'])

        # tableWidget 업데이트
        self.tableWidget.setRowCount(len(df))
        self.tableWidget.setColumnCount(len(df.columns))
        self.tableWidget.setHorizontalHeaderLabels(df.columns)

        for row_index, row in enumerate(df.itertuples(index=False)):
            for col_index, value in enumerate(row):
                item = QTableWidgetItem(str(value))
                self.tableWidget.setItem(row_index, col_index, item)



if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # config 파일에서 host 정보를 읽어옴
    config_path = 'db/config/config.ini'
    config = ConfigParser()
    config.read(config_path)
    
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