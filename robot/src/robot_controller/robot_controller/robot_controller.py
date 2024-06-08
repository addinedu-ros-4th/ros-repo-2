import rclpy as rp
import os
import time
import math
from rclpy.node import Node
from copy import deepcopy

from task_msgs.srv import AllocateTask

from task_msgs.srv import ArucoCommandResponse
from task_msgs.srv import StepControlResponse
from task_msgs.srv import CompletePickingResponse

from task_msgs.srv import ArucoCommand
from task_msgs.srv import StepControl
from task_msgs.srv import CompletePicking

from task_msgs.msg import TaskCompletion
from task_msgs.msg import CurrentPath
from task_msgs.msg import RobotStatus
from task_msgs.msg import OutTask

from std_msgs.msg import Empty
from std_msgs.msg import Float32
from rclpy.executors import MultiThreadedExecutor

from nav2_simple_commander.robot_navigator import BasicNavigator
import numpy as np
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import TaskResult
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped



class My_Location(Node) : 
    def __init__(self, controller):
        super().__init__("robot_sub_controller")
        self.controller = controller
        
        self.obstacle_1 = [[2, 2],[2, 2]]
        self.obstacle_2 = [[2, 2],[2, 2]]
        # publisher
        self.publisher_out_task = self.create_publisher(
            OutTask,
            "/out_task",
            10
        )

        self.publisher_current_path = self.create_publisher(
            CurrentPath,
            "/obstacle",
            10
        )

        # subscriber
        self.subcription_amclpose = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.current_pose,
            10
        )

        self.subscription_emergency_stop = self.create_subscription(
            Empty,
            "/emergency_stop",
            self.emergency_button_is_clicked,
            10
        )

        self.subscription_current_path = self.create_subscription(
            CurrentPath,
            "/obstacle_1",
            self.renew_map_status_1,
            10
        )

        self.subscription_current_path = self.create_subscription(
            CurrentPath,
            "/obstacle_2",
            self.renew_map_status_2,
            10
        )

        # server
        self.server = self.create_service(
            CompletePicking, 
            "/complete_picking", 
            self.next_out_is_clicked
        )

        self.arucomarker_server = self.create_service(
            ArucoCommandResponse, 
            "/aruco_command_response", 
            self.aruco_is_done
        )

        self.lift_server = self.create_service(
            StepControlResponse, 
            "/step_control_response", 
            self.lift_is_done
        )

        self.timer = self.create_timer(5.0, self.renew_out_task_data)
        self.path_timer = self.create_timer(0.1, self.send_path_status)
        

    def renew_map_status_1(self, data):
        
        self.obstacle_1[0][0] = data.start_x
        self.obstacle_1[0][1] = data.start_y
        self.obstacle_1[1][0] = data.end_x
        self.obstacle_1[1][1] = data.end_y

        self.send_map_status()


    def renew_map_status_2(self, data):
        self.obstacle_2[0][0] = data.start_x
        self.obstacle_2[0][1] = data.start_y
        self.obstacle_2[1][0] = data.end_x
        self.obstacle_2[1][1] = data.end_y

        self.send_map_status()


    def send_map_status(self):
        map_data = deepcopy(self.controller.is_passable_list)
        if self.obstacle_1[0][0] == -1:
            map_data[self.obstacle_1[0][0]][self.obstacle_1[0][1]] = True
            map_data[self.obstacle_1[1][0]][self.obstacle_1[1][1]] = True
        else:
            map_data[self.obstacle_1[0][0]][self.obstacle_1[0][1]] = False
            map_data[self.obstacle_1[1][0]][self.obstacle_1[1][1]] = False

        if self.obstacle_2[0][0] == -1:
            map_data[self.obstacle_2[0][0]][self.obstacle_2[0][1]] = True
            map_data[self.obstacle_2[1][0]][self.obstacle_2[1][1]] = True
        else:
            map_data[self.obstacle_2[0][0]][self.obstacle_2[0][1]] = False
            map_data[self.obstacle_2[1][0]][self.obstacle_2[1][1]] = False
        
        self.controller.current_is_passable_list = deepcopy(map_data)


    def send_path_status(self):
        self.publisher_current_path.publish(self.controller.current_path_msg)

        
    def renew_out_task_data(self):
        msg = OutTask()
        msg.location = self.controller.current_task_location
        msg.product = self.controller.item
        msg.count = self.controller.quantity
        
        self.publisher_out_task.publish(msg)


    def current_pose(self, data):
        self.controller.my_pose[0] = data.pose.pose.position.x
        self.controller.my_pose[1] = data.pose.pose.position.y

        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        
        roll, pitch, yaw = self.euler_from_quaternion(x, y, z, w)
        self.controller.my_yaw  = yaw

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z


    def next_out_is_clicked(self, req, res):
        self.controller.next_out = True
        return res


    def emergency_button_is_clicked(self, data):
        self.controller.task_status = "emergency"


    def aruco_is_done(self, req, res):
        self.controller.marker_service_done = True
        self.get_logger().info("marker end")
        return res


    def lift_is_done(self, req, res):
        self.controller.lift_service_done = True
        self.get_logger().info("lift end")
        return res



ID = os.getenv('ROS_DOMAIN_ID', 'Not set')
class RobotController(Node) : 
    def __init__(self) : 
        super().__init__("robot_controller")
        self.get_logger().info("start robot_controller")
        self.nav = BasicNavigator()
        
        self.current_task_location  = ""
        self.tasking                = False
        self.next_out               = False
        self.lift_service_done      = False
        self.marker_service_done    = False
        
        self.target_type    = ""
        self.task_status    = ""
        self.task_id        = ""
        self.item           = ""
        self.quantity       = 0
        self.my_pose        = [0.0, 0.0, 0.0]
        self.my_yaw         = 0
        
        self.current_path_msg = CurrentPath()


        self.server = self.create_service(
            AllocateTask, 
            f"/allocate_task_{ID}", 
            self.task_callback
        )

        
        self.task_completion_publisher = self.create_publisher(
            TaskCompletion,
            "/task_completion",
            10
        )

        self.robot_status_publisher = self.create_publisher(
            RobotStatus,
            "/robot_status",
            10
        )

        self.current_path_publisher = self.create_publisher(
            CurrentPath,
            "/current_path",
            10
        )

        # self.wait_client = self.create_client(

        # )

        self.lift_client = self.create_client(
            StepControl, "/step_control"
        )

        self.arucomarker_client = self.create_client(
            ArucoCommand, "/aruco_control"
        )

        self.complete_picking_client = self.create_client(
            CompletePickingResponse, "/complete_picking_response"
        )

        # pose x, y, z
        # 추후 json or yaml로 변경 필요
        self.POSE_DICT = {
            "I1" : [0.4, -1.425, 0.0],  "I2" : [0.4, -0.925, 0.0],  "I3" : [0.4, -0.625, 0.0],
            "O1" : [1.45, 1.06, 0.0],   "O2" : [1.45, 0.66, 0.0],   "O3" : [1.45, 0.46, 0.0],
            "P1" : [0.4, 1.16, 0.0],    "P2" : [0.4, 0.58, 0.0],    "P3" : [0.4, 0.0, 0.0],
            "R1" : [1.45, -1.2, 0.0],   "R2" : [1.45, -0.9, 0.0],
            "A1" : [0.7, 0.3, 0.0],    "A1_2" : [0.7, 0.3, 0.0], 
            "A2" : [1.15, 0.3, 0.0],    "A2_2" : [1.15, 0.3, 0.0],
            "B1" : [0.7, -0.5, 0.0],   "B1_2" : [0.7, -0.5, 0.0], 
            "B2" : [1.15, -0.5, 0.0],   "B2_2" : [1.15, -0.5, 0.0],
            "C1" : [0.7, -1.2, 0.0],   "C1_2" : [0.7, -1.2, 0.0], 
            "C2" : [1.15, -1.2, 0.0],   "C2_2" : [1.15, -1.2, 0.0]
        }

        self.YAW_DICT = {
            "I1" : 3.14,    "I2" : 3.14,    "I3" : 3.14,
            "O1" : 0.0,     "O2" : 0.0,     "O3" : 0.0,
            "P1" : 3.14,    "P2" : 3.14,    "P3" : 3.14,
            "A1" : 1.57,    "A1_2" : 1.57, 
            "A2" : 1.57,    "A2_2" : 1.57,
            "B1" : 1.57,    "B1_2" : 1.57, 
            "B2" : 1.57,    "B2_2" : 1.57, 
            "C1" : 1.57,    "C1_2" : 1.57, 
            "C2" : 1.57,    "C2_2" : 1.57, 
            "R1" : 0.0,     "R2" : 0.0
        }

        self.declare_lists()
        
        
        self.send_robot_status_topics()
        self.get_logger().info("controller is ready")


    def declare_lists(self):
        self.PATH_LIST = [
            [[0.3, -1.1, 0.0],  [0.8, -1.1, 0.0],  [1.2, -1.1, 0.0],  [1.5, -1.1, 0.0]],
            [[0.3, -0.75, 0.0], [0.8, -0.75, 0.0], [1.2, -0.75, 0.0], [1.5, -0.75, 0.0]],
            [[0.3, -0.4, 0.0],  [0.8, -0.4, 0.0],  [1.2, -0.4, 0.0],  [1.5, -0.4, 0.0]],
            [[0.3, 0.0, 0.0],   [0.8, 0.0, 0.0],   [1.2, 0.0, 0.0],   [1.5, 0.0, 0.0]],
            [[0.3, 0.4, 0.0],   [0.8, 0.4, 0.0],   [1.2, 0.4, 0.0],   [1.5, 0.4, 0.0]],
            [[0.3, 0.75, 0.0],  [0.8, 0.75, 0.0],  [1.2, 0.75, 0.0],  [1.5, 0.75, 0.0]],
            [[0.3, 1.0, 0.0],   [0.8, 1.0, 0.0],   [1.2, 1.0, 0.0],   [1.5, 1.0, 0.0]]
        ]
        
        self.X_LIST = []
        self.Y_LIST = []
        for i in self.PATH_LIST[0]:
            self.X_LIST.append(i[0])
        for i in self.PATH_LIST:
            self.Y_LIST.append(i[0][1])

        self.is_passable_list = [[True for _ in range(len(self.PATH_LIST[0]))] for _ in range(len(self.PATH_LIST))]
        not_passable_index_list = [[1, 1], [1, 2], [3, 1], [3, 2], [5, 1], [5, 2]]
        
        for i in not_passable_index_list:
            self.is_passable_list[i[0]][i[1]] = False

        self.current_is_passable_list = deepcopy(self.is_passable_list)


    def check_emergency_status(self):
        while self.task_status == "emergency":
            try:
                self.nav.cancelTask()
            except:
                pass
            self.get_logger().info("emergency!!!!!!!!!!")
            time.sleep(1)


    def send_robot_status_topics(self):
        msg = RobotStatus()

        
        msg.robot_id = ID
        msg.robot_status = "available"

        for i in range(20):
            self.robot_status_publisher.publish(msg)
            time.sleep(0.1)


    def task_callback(self, req, res) :
        self.get_logger().info("task_list:" + req.location)
        if not self.tasking:
            try:
                self.item                   = req.item
                self.tasking                = True
                self.task_id                = req.task_id
                self.quantity               = req.quantity
                self.task_status            = req.task_type
                self.current_task_location  = req.location
                
                res.success     = self.follow_path(self.current_task_location, req.lift)
                self.tasking    = False

                self.send_complete_task_topics()

            except Exception as e:
                self.tasking = False
                self.get_logger().error(f"{e}")
                
        else:
            res.success = False

        return res
    
    
    def send_complete_task_topics(self):
        msg = TaskCompletion()

        msg.success     = True
        msg.robot_id    = ID
        msg.task_id     = self.task_id

        for i in range(20):
            self.task_completion_publisher.publish(msg)
            time.sleep(0.1)


    def follow_path(self, pose_name, lift) :
        try:
            self.get_logger().info("path_planning start")
            
            # for pose_name in pose_list:
            target_pose = self.POSE_DICT[pose_name]
            target_yaw  = self.YAW_DICT[pose_name]
            self.target_type = "Stopover"
            self.real_time_stopover_planning(target_pose)
            
            self.get_logger().info(f"goto{pose_name}")
            self.target_type = "Main"
            self.move_pose(target_pose, target_yaw)

            if lift == "Up" : # lift up first place (첫 장소 리프트 업)
                self.get_logger().info("lift up")
                # self.service_call_lift(pose_name, "down")
                # self.service_call_marker(pose_name, "forward")
                # self.service_call_lift(pose_name, "up")
                # self.service_call_marker(pose_name, "backward")

            elif lift == "Down" : # lift down last place(마지막 장소 리프트 다운)
                self.get_logger().info("lift down")
                # self.service_call_lift(pose_name, "up")
                # self.service_call_marker(pose_name, "forward")
                # self.service_call_lift(pose_name, "down")
                # self.service_call_marker(pose_name, "backward")

            else : # 나머지 장소
                self.current_task_location = pose_name
                self.checking_task_is_out() 
                
            self.get_logger().info("move end")
            return True
        
        except Exception as e:
            self.get_logger().error(f"{e}")
            return False
        

    def checking_task_is_out(self):
        if self.task_status == "OUT":
            while not self.next_out:
                time.sleep(1)
                self.get_logger().info("wait for button ...")
            self.next_out = False
            req = CompletePickingResponse.Request()
            self.complete_picking_client.call_async(req)


    def real_time_stopover_planning(self, target_pose):
        nearest_point_from_target, nearest_point_from_target_index = self.search_nearest_point(target_pose)
        nearest_point_from_me, nearest_point_from_me_index = self.search_nearest_point(self.my_pose)
        current_point_index = nearest_point_from_me_index.copy()
        
        self.get_logger().info(f"start stopover : {current_point_index} {nearest_point_from_target_index}")
        
        direction_robot_to_target = [
            1 if nearest_point_from_target_index[0] - nearest_point_from_me_index[0] > 0 else -1,
            1 if nearest_point_from_target_index[1] - nearest_point_from_me_index[1] > 0 else -1
        ]
        
        self.move_pose(nearest_point_from_me, 0.0)
        
        while current_point_index != nearest_point_from_target_index:
            direction_robot_to_target = [
                1 if nearest_point_from_target_index[0] - current_point_index[0] > 0 else -1,
                1 if nearest_point_from_target_index[1] - current_point_index[1] > 0 else -1
            ]
            passable_path = self.search_passable_path(current_point_index, direction_robot_to_target, nearest_point_from_target_index)
            if passable_path == []:
                self.get_logger().info("wait stopover")

                for i in range(0, 7):
                    self.get_logger().info(f"{self.current_is_passable_list[i]}")
                self.get_logger().info("current map end")
                
                for i in range(0, 7):
                    self.get_logger().info(f"{self.is_passable_list[i]}")
                self.get_logger().info("base map end")
                
                time.sleep(10)
                
            elif len(passable_path) == 2:
                self.get_logger().info("nomal move")
                current_point_index = self.move_set(passable_path, current_point_index)

            elif len(passable_path) > 2:
                self.get_logger().info(f"not nomal move{passable_path}")
                passable_path_list = [passable_path[:2], passable_path[2:4]]

                for path in passable_path_list:
                    current_point_index = self.move_set(path, current_point_index)

        self.current_path_msg.start_x = 1 # path에서 장애물 비킴
        self.current_path_msg.start_y = 3

    def search_nearest_point(self,target_pose): # 타겟에 가장 가까운 point 찾기
        nearest_point = [999, 999, 0.0]
        nearest_point_index = [0, 0]
        min_x_vel = 999
        min_y_vel = 999

        for index, vel in enumerate(self.X_LIST):
            distance = abs(target_pose[0] - vel)
            if distance < min_x_vel:
                min_x_vel               = distance
                nearest_point[0]        = vel
                nearest_point_index[1]  = index


        for index, vel in enumerate(self.Y_LIST):
            distance = abs(target_pose[1] - vel)
            if distance < min_y_vel:
                min_y_vel               = distance
                nearest_point[1]        = vel
                nearest_point_index[0]  = index
            
        return nearest_point, nearest_point_index


    def search_passable_path(self, current_point_index, direction_robot_to_target, target_point_index):
        x           = current_point_index[0]
        y           = current_point_index[1]
        target_x    = target_point_index[0]
        target_y    = target_point_index[1]
        return_vel  = []

        if y != target_y and self.current_is_passable_list[x][y + direction_robot_to_target[1]] and (0 <= y + direction_robot_to_target[1] < 4):
            return_vel = [x, y + direction_robot_to_target[1]]


        if x != target_x and self.current_is_passable_list[x + direction_robot_to_target[0]][y] and (0 <= x + direction_robot_to_target[0] < 7):
            return_vel = [x + direction_robot_to_target[0], y]


        if return_vel == [] and (y != target_y or x != target_x) : # 
            if 0 == y - target_y   and self.current_is_passable_list[x + direction_robot_to_target[0]][y + direction_robot_to_target[1]] and self.current_is_passable_list[x][y + direction_robot_to_target[1]]:
                return_vel = [x, y + direction_robot_to_target[1], x + direction_robot_to_target[0], y + direction_robot_to_target[1]]
                    
            elif 0 == y - target_y and self.current_is_passable_list[x + direction_robot_to_target[0]][y - direction_robot_to_target[1]] and self.current_is_passable_list[x][y - direction_robot_to_target[1]]:
                return_vel = [x, y - direction_robot_to_target[1], x + direction_robot_to_target[0], y - direction_robot_to_target[1]]
            
            elif 0 == x - target_x and self.current_is_passable_list[x + direction_robot_to_target[0]][y + direction_robot_to_target[1]] and self.current_is_passable_list[x + direction_robot_to_target[0]][y]:
                return_vel = [x + direction_robot_to_target[0], y, x + direction_robot_to_target[0], y + direction_robot_to_target[1]]

            elif 0 == x - target_x and self.current_is_passable_list[x - direction_robot_to_target[0]][y + direction_robot_to_target[1]] and self.current_is_passable_list[x - direction_robot_to_target[0]][y]:
                return_vel = [x - direction_robot_to_target[0], y, x - direction_robot_to_target[0], y + direction_robot_to_target[1]]

        return return_vel
        

    def move_set(self, passable_path, current_point_index):
        # self.get_logger().info("move stopover")
        self.get_logger().info(f"my pose = {current_point_index}")
        self.get_logger().info(f"go pose = {passable_path}")

        self.current_path_msg.start_x   = current_point_index[0]
        self.current_path_msg.start_y   = current_point_index[1]
        self.current_path_msg.end_x     = passable_path[0]
        self.current_path_msg.end_y     = passable_path[1]
        
        yaw = self.point_to_yaw(self.PATH_LIST[current_point_index[0]][current_point_index[1]],
                                self.PATH_LIST[passable_path[0]][passable_path[1]])
        
        self.move_pose(self.PATH_LIST[current_point_index[0]][current_point_index[1]], yaw)
        self.move_pose(self.PATH_LIST[passable_path[0]][passable_path[1]], yaw)
        
        current_point_index = passable_path.copy()
        
        return current_point_index


    def find_approximation_to_pose_list(self, pose_list, target_vel):
        min_vel     = 999
        min_index   = 0

        for i, X in enumerate(pose_list):
            vel = abs(target_vel - X)
        
            if vel < min_vel:
                min_vel     = vel
                min_index   = i

        return min_index

        
    def point_to_yaw(self, start_pose, target):
        x = target[0] - start_pose[0]
        y = target[1] - start_pose[1]
        rad = math.atan2(y, x)

        return rad
    
        
    def service_call_lift(self, pose_name, direction) :
        if "_" in pose_name:
            floor = int(pose_name.split("_")[1])
        else :
            floor = 1
        
        req = StepControl.Request()
        req.floor = floor
        req.direction = direction

        self.get_logger().info(f"req.floor : {req.floor}")
        self.get_logger().info(f"req.direction : {req.direction}")

        res = self.lift_client.call_async(req)
        self.wait_lift_res()
        

    def wait_lift_res(self):
        while not self.lift_service_done:
            time.sleep(1)
        self.lift_service_done = False


    def service_call_marker(self, location=None ,direction=None):
        self.get_logger().info(direction)
        if "_" in location:
            location = location.split("_")[0]

        req = ArucoCommand.Request()
        req.location    = location
        req.direction   = direction

        res = self.arucomarker_client.call_async(req)
        self.wait_marker_res()
        

    def wait_marker_res(self):
        while not self.marker_service_done:
            time.sleep(1)
            # self.get_logger().info("wait marker response")
        self.marker_service_done = False


    def move_pose(self, target_pose, yaw) :
        q = self.euler_to_quaternion(yaw=yaw)

        self.get_logger().info(f"pose : {target_pose} yaw : {yaw}")
        goal_pose       = PoseStamped()
        redeem_vector   = self.redeem_pose(target_pose, 0.25)
        
        goal_pose.header.frame_id   = 'map'
        goal_pose.header.stamp      = self.nav.get_clock().now().to_msg()

        goal_pose.pose.position.x = target_pose[0] + redeem_vector[0]
        goal_pose.pose.position.y = target_pose[1] + redeem_vector[1]
        goal_pose.pose.position.z = target_pose[2] + redeem_vector[2]

        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        self.nav.goToPose(goal_pose)

        self.nav_distance_feedback(yaw)


    def euler_to_quaternion(self, yaw = 0, pitch = 0, roll = 0):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]
    

    def redeem_pose(self, target_pose, distance):
        x                   = target_pose[0]
        y                   = target_pose[1]
        current_x           = self.my_pose[0]
        current_y           = self.my_pose[1]
        direction_vector    = [x - current_x, y - current_y, 0.0]

        magnitude = math.sqrt(direction_vector[0]**2 + direction_vector[1]**2 + direction_vector[2]**2)

        if magnitude == 0:
            raise ValueError 
        
        unit_vector     = [direction_vector[0] / magnitude, direction_vector[1] / magnitude, direction_vector[2] / magnitude]
        redeem_vector   = [unit_vector[0] * distance, unit_vector[1] * distance, unit_vector[2] * distance]

        return redeem_vector


    def nav_distance_feedback(self, target_yaw) :
        if self.target_type == "Main":
            sec = 0
        else:
            sec = 0.5
        i = 0
        send_data = Float32()
        while not self.nav.isTaskComplete():
            self.check_emergency_status()
            i = i + 1
            feedback = self.nav.getFeedback()
            self.current_path_publisher.publish(self.current_path_msg)
            if feedback and i % 30 == 0 :
                self.get_logger().info("distance remaining: " + "{:.2f}".format(feedback.distance_remaining) + " meters.")
                send_data.data = feedback.distance_remaining
                
                if (feedback.distance_remaining <= 0.15 and feedback.distance_remaining != 0.0 and 0.1 > (self.my_yaw - target_yaw)) or Duration.from_msg(feedback.navigation_time) > Duration(seconds=40.0) :
                    time.sleep(sec) # 목표지점 인접시 지연후 다음목적지
                    self.nav.cancelTask()
                    self.get_logger().info("cancel nav Task")

                    return


def main(args = None) : 
    rp.init(args=args)
    
    con = RobotController()
    sub = My_Location(controller = con)


    excutor = MultiThreadedExecutor()

    excutor.add_node(con)
    excutor.add_node(sub)
    
    try : 
        excutor.spin()
    finally : 
        excutor.shutdown()
        sub.destroy_node()
        con.destroy_node()
        rp.shutdown()

    


if __name__ == "__main__" : 
    main()