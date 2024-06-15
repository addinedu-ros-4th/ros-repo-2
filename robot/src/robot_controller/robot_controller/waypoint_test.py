import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.QtCore import *

from rclpy.executors import MultiThreadedExecutor
from threading import Thread
from nav2_simple_commander.robot_navigator import BasicNavigator
from PyQt5.QtCore import pyqtSignal
import os 
import yaml 
import sys 
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from task_msgs.srv import ArucoCommand
from task_msgs.srv import StepControl

from_class = uic.loadUiType("/home/addinedu/dev_ws/Final_project/ros-repo-2/robot/src/robot_controller/ui/qt_controll.ui")[0]

class StepController(Node):
    def __init__(self, ui):
        super().__init__('step_node')
        self.ui = ui
        self.ui.stepClicked.connect(self.step)

        self.lift_client = self.create_client(StepControl, "/step_control")

    def step(self):
        req = StepControl.Request()
        req.floor = int(self.ui.stepFloor.currentText())
        req.direction = self.ui.stepDirection.currentText()
        res = self.lift_client.call_async(req)


class ArucoController(Node):  # Node 상속
    def __init__(self, ui):
        super().__init__('aruco_node')  # super() 호출하여 Node 초기화
        self.ui = ui
        self.ui.arucoClicked.connect(self.aruco)
        self.aruco_client = self.create_client(ArucoCommand, "/aruco_control")

    def aruco(self):
        req = ArucoCommand.Request()
        req.location = self.ui.arucoLocation.currentText()
        req.direction = self.ui.arucoDirection.currentText()
        res = self.aruco_client.call_async(req)  # aruco_client 사용


class RobotGoal(Node):
    def __init__(self, ui):
        super().__init__("goal_node")
        self.ui = ui
        self.ui.stepClicked.connect(self.cancel_goal)
        self.ui.arucoClicked.connect(self.cancel_goal)
        self.ui.goalClicked.connect(self.go_to_goal)
        
        self.nav = BasicNavigator()
        self.goal_pose = PoseStamped()

    def cancel_goal(self):
        self.nav.cancelTask()

    def go_to_goal(self):
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = float(self.ui.goalXEdit.text())
        self.goal_pose.pose.position.y = float(self.ui.goalYEdit.text())
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = float(self.ui.orienZEdit.text())
        self.goal_pose.pose.orientation.w = float(self.ui.orienWEdit.text())
        self.nav.goToPose(self.goal_pose)


class ControlWidget(QWidget, from_class):
    goalClicked = pyqtSignal(bool)
    stepClicked = pyqtSignal(bool)
    arucoClicked = pyqtSignal(bool)

    def __init__(self):
        super().__init__()

        self.setupUi(self)
        self.setWindowTitle("test")
        self.goalXEdit.setText("0.0")
        self.goalYEdit.setText("0.0")
        self.orienXEdit.setText("0.0")
        self.orienYEdit.setText("0.0")
        self.orienZEdit.setText("0.0")
        self.orienWEdit.setText("0.0")

        self.PATH_LIST = {
            'w1' : [0.5, -1.1], 'w2' : [0.5, -0.5], 'w3' : [0.5, 0.3], 'w4' : [0.5, 1.0],
            'w5' : [1.55, -1.1], 'w6' : [1.55, -0.5], 'w7' : [1.55, 0.3], 'w8' : [1.55, 1.0],
        }
        
        self.POSITION_DICT = {
            "I1" : [0.5, -1.425],  "I2" : [0.5, -1.1],  "I3" : [0.5, -0.625],
            "O1" : [1.45, 1.06],   "O2" : [1.45, 0.66],   "O3" : [1.45, 0.46],
            "P3" : [0.3, 1.2],    "P2" : [0.3, 0.7],    "P1" : [0.3, 0.15],
            "R1" : [1.45, -1.4],   "R2" : [1.45, -1.1],
            "A1" : [0.9, 0.3],    "A1_2" : [0.9, 0.3], 
            "A2" : [1.37, 0.3],    "A2_2" : [1.37, 0.3],
            "B1" : [0.9, -0.5],   "B1_2" : [0.9, -0.5], 
            "B2" : [1.37, -0.5],   "B2_2" : [1.37, -0.5],
            "C1" : [0.9, -1.2],   "C1_2" : [0.9, -1.2], 
            "C2" : [1.37, -1.2],   "C2_2" : [1.37, -1.2]
        }

        self.YAW_DICT = {
            "up" : 3.14,    
            "down" : 0.0,    
            "right" : 1.57, 
            "left" : -1.57,
        }

        # set event
        self.btnGo.clicked.connect(self.go_to_goal)
        self.pathCombo.currentIndexChanged.connect(self.path_changed)
        self.pointCombo.currentIndexChanged.connect(self.postion_changed)
        self.orienCombo.currentIndexChanged.connect(self.orien_changed)
        self.btnAruco.clicked.connect(self.aruco_control)
        self.btnStep.clicked.connect(self.step_control)


    def path_changed(self):
        path = self.pathCombo.currentText()
        coordinates = self.PATH_LIST[path]
        self.goalXEdit.setText(str(coordinates[0]))
        self.goalYEdit.setText(str(coordinates[1]))
        

    def postion_changed(self):
        position = self.pointCombo.currentText()
        coordinates = self.POSITION_DICT[position]
        self.goalXEdit.setText(str(coordinates[0]))
        self.goalYEdit.setText(str(coordinates[1]))

    def orien_changed(self):
        orientation = self.orienCombo.currentText()
        yaw = self.YAW_DICT.get(orientation, 0.0)
        quaternion = quaternion_from_euler(0, 0, yaw)

        self.orienXEdit.setText(str(quaternion[0]))
        self.orienYEdit.setText(str(quaternion[1]))
        self.orienZEdit.setText(str(quaternion[2]))
        self.orienWEdit.setText(str(quaternion[3]))


    def go_to_goal(self):
        self.goalClicked.emit(True)

    def aruco_control(self):
        self.arucoClicked.emit(True)

    def step_control(self):
        self.stepClicked.emit(True)


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    app = QApplication(sys.argv)
    myWindows = ControlWidget()
    myWindows.show()

    goal_node = RobotGoal(myWindows)
    step_node = StepController(myWindows)
    aruco_node = ArucoController(myWindows)

    executor.add_node(step_node)
    executor.add_node(aruco_node)
    executor.add_node(goal_node)

    thread = Thread(target=executor.spin)
    thread.start()

    try:
        app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()