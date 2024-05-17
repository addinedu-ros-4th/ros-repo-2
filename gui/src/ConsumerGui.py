import sys
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QPushButton, QVBoxLayout, QWidget, QMainWindow
from websocket import create_connection
import json

from_class = uic.loadUiType("gui/ui/ConsumerGui.ui")[0]

class ConsumerGui(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.initUI()


    def initUI(self):
        self.setWindowTitle('Consumer GUI')
        self.orderBtn.clicked.connect(self.send_task_to_ros)
        self.show()


    def send_task_to_ros(self):
        # Create a WebSocket connection to the rosbridge
        ws = create_connection("ws://192.168.0.85:9090")
        
        orders = {
            "user_id": "123",
            "items": ["cola", "water"],
            "quantities": [1, 2]
        }
        
        order_message = json.dumps({
            "op": "publish",
            "topic": "/order",
            "msg": {"data": json.dumps(orders)}
        })
                                
        # Send the task message
        ws.send(order_message)
        ws.close()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = ConsumerGui()
    sys.exit(app.exec_())
