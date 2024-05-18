# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'orderIYjGSt.ui'
##
## Created by: Qt User Interface Compiler version 5.15.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################


from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5 import uic
import sys 
import json
import os

from_orderpage_class = uic.loadUiType("order.ui")[0]

class Ui_OrderWindow(QMainWindow, from_orderpage_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Order Page")
        
        self.num_value = 0  # 숫자 값을 저장하는 변수
        self.user_id = 0  # 유저 아이디를 저장하는 변수
        self.num.setText(str(self.num_value))  # 초기값 설정
        self.num.setReadOnly(True)  # QLineEdit을 읽기 전용으로 설정
        
        self.plus.clicked.connect(self.increase_num)
        self.minus.clicked.connect(self.decrease_num)
        self.add_btn.clicked.connect(self.add_to_list)
        self.buy_btn.clicked.connect(self.save_to_json)
        self.delete_btn.clicked.connect(self.delete_from_list)
        
        self.model = QStandardItemModel(self.listView)  # QStandardItemModel 생성
        self.listView.setModel(self.model)  # QListView에 모델 설정
        
        # 기존 JSON 파일에서 user_id를 불러옵니다.
        self.load_user_id()
        
    def load_user_id(self):
        if os.path.exists("order_data.json"):
            with open("order_data.json", "r", encoding="utf-8") as file:
                data = json.load(file)
                if isinstance(data, list) and data:  # 데이터가 리스트이고 비어 있지 않으면
                    self.user_id = data[-1].get("user_id", -1) + 1
                    
    def increase_num(self):
        self.num_value += 1
        self.num.setText(str(self.num_value))
        
    def decrease_num(self):
        if self.num_value > 0:
            self.num_value -= 1
        self.num.setText(str(self.num_value))
        
    def delete_from_list(self):
        selected_index = self.listView.selectedIndexes()
        if selected_index:
            index = selected_index[0]
            self.model.removeRow(index.row())
        
    def add_to_list(self):
        product_name = self.select.currentText()  # 선택된 상품명 가져오기
        quantity = self.num.text()  # 수량 가져오기
        list_item = QStandardItem(f"{product_name}: {quantity}")
        self.model.appendRow(list_item)  # 모델에 항목 추가
        
    def save_to_json(self):
        new_data = {
            "user_id": self.user_id,
            "product_name": [],
            "quantity": []
        }

        for row in range(self.model.rowCount()):
            item = self.model.item(row).text()
            product_name, quantity = item.split(": ")
            new_data["product_name"].append(product_name)
            new_data["quantity"].append(int(quantity))
        
        # 기존 데이터 불러오기
        if os.path.exists("order_data.json"):
            with open("order_data.json", "r", encoding="utf-8") as file:
                existing_data = json.load(file)
                if not isinstance(existing_data, list):
                    existing_data = []
        else:
            existing_data = []

        # 동일 user_id가 있는지 확인하고, 있으면 병합
        for data in existing_data:
            if data["user_id"] == new_data["user_id"]:
                for product, qty in zip(new_data["product_name"], new_data["quantity"]):
                    if product in data["product_name"]:
                        index = data["product_name"].index(product)
                        data["quantity"][index] += qty
                    else:
                        data["product_name"].append(product)
                        data["quantity"].append(qty)
                break
        else:
            existing_data.append(new_data)

        # 업데이트된 데이터를 JSON 파일에 저장
        with open("order_data.json", "w", encoding="utf-8") as file:
            json.dump(existing_data, file, ensure_ascii=False, indent=4)
        
        QMessageBox.information(self, "Saved", "결제완료")
        
        # user_id 증가
        self.user_id += 1
        
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
        self.select.setStyleSheet(u"background-color: rgb(255, 255, 255);\n"
"\n"
"")
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
        self.minus.setStyleSheet(u"\n"
"	border:none;\n"
"")
        self.plus = QPushButton(self.centralwidget)
        self.plus.setObjectName(u"plus")
        self.plus.setGeometry(QRect(550, 180, 41, 25))
        self.plus.setFont(font)
        self.plus.setStyleSheet(u"\n"
"	border:none;\n"
"")
        self.buy_btn = QPushButton(self.centralwidget)
        self.buy_btn.setObjectName(u"buy_btn")
        self.buy_btn.setGeometry(QRect(560, 410, 89, 51))
        self.buy_btn.setStyleSheet(u"\n"
"	color:#000;\n"
"	border:none;\n"
"")
        icon = QIcon()
        icon.addFile(u"/home/addinedu/Downloads/free-icon-buy-button-6064456.png", QSize(), QIcon.Normal, QIcon.Off)
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
        self.home.setStyleSheet(u"background-color: rgb(255, 255, 255);\n"
"border-radius: 30px")
        icon1 = QIcon()
        icon1.addFile(u"/home/addinedu/Downloads/home.png", QSize(), QIcon.Normal, QIcon.Off)
        self.home.setIcon(icon1)
        self.home.setIconSize(QSize(25, 25))
        self.order = QPushButton(self.Wmenu)
        self.order.setObjectName(u"order")
        self.order.setGeometry(QRect(10, 220, 61, 61))
        self.order.setStyleSheet(u"background-color: rgb(255, 255, 255);\n"
"border-radius: 30px")
        icon2 = QIcon()
        icon2.addFile(u"/home/addinedu/Downloads/pngwing.com.png", QSize(), QIcon.Normal, QIcon.Off)
        self.order.setIcon(icon2)
        self.order.setIconSize(QSize(30, 30))
        self.chart = QPushButton(self.Wmenu)
        self.chart.setObjectName(u"chart")
        self.chart.setGeometry(QRect(10, 350, 61, 61))
        self.chart.setStyleSheet(u"background-color: rgb(255, 255, 255);\n"
"border-radius: 30px")
        icon3 = QIcon()
        icon3.addFile(u"/home/addinedu/Downloads/bar-chart.png", QSize(), QIcon.Normal, QIcon.Off)
        self.chart.setIcon(icon3)
        self.chart.setIconSize(QSize(30, 30))
        self.label = QLabel(self.Wmenu)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(20, 20, 41, 17))
        self.user = QPushButton(self.Wmenu)
        self.user.setObjectName(u"user")
        self.user.setGeometry(QRect(10, 480, 61, 61))
        self.user.setStyleSheet(u"background-color: rgb(255, 255, 255);\n"
"border-radius: 30px")
        icon4 = QIcon()
        icon4.addFile(u"/home/addinedu/Downloads/free-icon-user-482636.png", QSize(), QIcon.Normal, QIcon.Off)
        self.user.setIcon(icon4)
        self.user.setIconSize(QSize(30, 30))
        
        self.add_btn = QPushButton(self.centralwidget)
        self.add_btn.setObjectName(u"add_btn")
        self.add_btn.setGeometry(QRect(610, 180, 41, 31))
        self.add_btn.setStyleSheet(u"border:none;")
        
        icon5 = QIcon()
        icon5.addFile(u"/home/addinedu/Downloads/IMG_7031.png", QSize(), QIcon.Normal, QIcon.Off)
        self.add_btn.setIcon(icon5)
        self.add_btn.setIconSize(QSize(35, 35))
        self.delete_btn = QPushButton(self.centralwidget)
        self.delete_btn.setObjectName(u"delete_btn")
        self.delete_btn.setGeometry(QRect(210, 420, 41, 25))
        self.delete_btn.setStyleSheet(u"border:none;")
        icon6 = QIcon()
        icon6.addFile(u"/home/addinedu/Downloads/delete-button-svgrepo-com.png", QSize(), QIcon.Normal, QIcon.Off)
        self.delete_btn.setIcon(icon6)
        self.delete_btn.setIconSize(QSize(25, 26))
        MainWindow.setCentralWidget(self.centralwidget)
        # self.select.addItems(["cola", "water", "ramen"])

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
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
    # retranslateUi
if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    window = Ui_OrderWindow()
    window.show()
    sys.exit(app.exec())


