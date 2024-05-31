from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import pandas as pd

class Ui_Setup:
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
        self.userinfo.setObjectName(u"userinfo2")
        self.userinfo.setGeometry(QRect(580, 50, 81, 17))
        
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
        self.tableWidget.setStyleSheet(u"background-color: rgb(255, 255, 255);")
        
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
        self.select.addItem(QCoreApplication.translate("OrderWindow", u"Coke", None))
        self.select.addItem(QCoreApplication.translate("OrderWindow", u"Sprite", None))
        self.select.addItem(QCoreApplication.translate("OrderWindow", u"Chapagetti", None))
        self.select.addItem(QCoreApplication.translate("OrderWindow", u"Buldak", None))
        self.select.addItem(QCoreApplication.translate("OrderWindow", u"Robot Vacuum", None))
        self.select.addItem(QCoreApplication.translate("OrderWindow", u"Coffee Pot", None))
        self.add_btn.setText("")

        self.delete_btn.setText("")
        self.stats.setText(QCoreApplication.translate("MainWindow", u"", None))
        self.userinfo.setText(QCoreApplication.translate("MainWindow", f"user id : {self.user_id}", None))
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

    