import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
import json
from websocket import create_connection

from_class = uic.loadUiType("gui/ui/test2.ui")[0]

class WindowClass(QMainWindow, from_class) :
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        # 버튼 초기화 및 설정
        self.btn = QPushButton('Click me', self)
        self.btn.clicked.connect(self.on_click)

        # 레이아웃 설정
        layout = QVBoxLayout()
        layout.addWidget(self.btn)
        self.setLayout(layout)
        self.show()


    def on_click(self):
        # 웹소켓 연결 생성
        ws = create_connection("ws://192.168.0.85:8080/")
        print("Sending 'Button Clicked'")
        data = json.dumps({
                    "deviceId": "GUI",
                    "message": "Button Clicked"
                })
        ws.send(data)
        result = ws.recv()
        print("Received from server: " + result)
        ws.close()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    myWindows = WindowClass()
    
    myWindows.show()
    
    sys.exit(app.exec_())