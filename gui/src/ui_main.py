# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'mainqilLGf.ui'
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
import os
from ui_order import Ui_OrderWindow

from_mainpage_class = uic.loadUiType("gui/ui/main.ui")[0]

class Ui_MainWindow(QMainWindow, from_mainpage_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Main")
        
        self.go_order.clicked.connect(self.open_order_window)
   
    def open_order_window(self):
        self.order_window = Ui_OrderWindow()
        self.order_window.show()
       
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(800, 600)
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
        self.home.setStyleSheet(u"background-color: rgb(255, 255, 255);\n"
"border-radius: 30px")
        icon = QIcon()
        icon.addFile(u"gui/image/home.png", QSize(), QIcon.Normal, QIcon.Off)
        self.home.setIcon(icon)
        self.home.setIconSize(QSize(25, 25))
        self.order = QPushButton(self.Wmenu)
        self.order.setObjectName(u"order")
        self.order.setGeometry(QRect(10, 220, 61, 61))
        self.order.setStyleSheet(u"background-color: rgb(255, 255, 255);\n"
"border-radius: 30px")
        icon1 = QIcon()
        icon1.addFile(u"gui/image/home.png", QSize(), QIcon.Normal, QIcon.Off)
        self.order.setIcon(icon1)
        self.order.setIconSize(QSize(30, 30))
        self.chart = QPushButton(self.Wmenu)
        self.chart.setObjectName(u"chart")
        self.chart.setGeometry(QRect(10, 350, 61, 61))
        self.chart.setStyleSheet(u"background-color: rgb(255, 255, 255);\n"
"border-radius: 30px")
        icon2 = QIcon()
        icon2.addFile(u"gui/image/home.png", QSize(), QIcon.Normal, QIcon.Off)
        self.chart.setIcon(icon2)
        self.chart.setIconSize(QSize(30, 30))
        self.label = QLabel(self.Wmenu)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(20, 20, 41, 17))
        self.user = QPushButton(self.Wmenu)
        self.user.setObjectName(u"user")
        self.user.setGeometry(QRect(10, 480, 61, 61))
        self.user.setStyleSheet(u"background-color: rgb(255, 255, 255);\n"
"border-radius: 30px")
        icon3 = QIcon()
        icon3.addFile(u"gui/image/home.png", QSize(), QIcon.Normal, QIcon.Off)
        self.user.setIcon(icon3)
        self.user.setIconSize(QSize(30, 30))
        self.Worder = QWidget(self.centralwidget)
        self.Worder.setObjectName(u"Worder")
        self.Worder.setGeometry(QRect(80, 150, 721, 281))
        self.Worder.setStyleSheet(u"background-color: rgb(82, 98, 116);")
        self.go_order = QPushButton(self.Worder)
        self.go_order.setObjectName(u"go_order")
        self.go_order.setGeometry(QRect(270, 110, 121, 61))
        font = QFont()
        font.setBold(True)
        font.setWeight(75)
        self.go_order.setFont(font)
        self.go_order.setStyleSheet(u"background-color: rgb(255, 255, 255);\n"
"border-radius: 10px")
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.home.setText("")
        self.order.setText("")
        self.chart.setText("")
        self.label.setText(QCoreApplication.translate("MainWindow", u"menu", None))
        self.user.setText("")
        self.go_order.setText(QCoreApplication.translate("MainWindow", u"\uc8fc\ubb38\ud558\uae30", None))
    # retranslateUi

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    window = Ui_MainWindow()
    window.show()
    sys.exit(app.exec())