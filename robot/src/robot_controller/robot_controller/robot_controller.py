import rclpy as rp
import os
import time
import math
from rclpy.node import Node

from task_msgs.srv import AllocateTask

from task_msgs.srv import ArucoCommandResponse
from task_msgs.srv import StepControlResponse
from task_msgs.srv import CompletePickingResponse

from task_msgs.srv import ArucoCommand
from task_msgs.srv import StepControl
from task_msgs.srv import CompletePicking

from task_msgs.msg import TaskCompletion
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
        
        self.publisher_out_task = self.create_publisher(
            OutTask,
            "/out_task",
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

        self.timer = self.create_timer(1.0, self.renew_out_task_data)
        
        
        
    def renew_out_task_data(self):
        msg = OutTask()
        msg.location = self.controller.current_task_location
        msg.product = self.controller.item
        msg.count = self.controller.quantity
        
        self.publisher_out_task.publish(msg)

    def current_pose(self, data):
        self.controller.my_pose[0] = data.pose.pose.position.x
        self.controller.my_pose[1] = data.pose.pose.position.y

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
        
        self.current_task_location = ""
        self.tasking = False
        self.next_out = False
        self.lift_service_done = False
        self.marker_service_done = False
        
        self.task_status = ""
        self.task_id = ""
        self.item = ""
        self.quantity = 0
        self.my_pose = [0.0, 0.0, 0.0]
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

        self.feedback_publisher = self.create_publisher(
            Float32,
            "/feedback",
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
            "I1" : [ 0.3, -1.225, 0.0], "I2" : [0.3, -0.925, 0.0], "I3" : [0.3, -0.625, 0.0],
            "O1" : [1.55, 1.06, 0.0], "O2" : [1.55, 0.76, 0.0], "O3" : [1.55, 0.46, 0.0],
            "P1" : [0.3, 0.0, 0.0], "P2" : [0.3, 0.58, 0.0], "P3" : [0.3, 1.16, 0.0],
            "A1" : [0.91, -0.12, 0.0], "A1_2" : [0.91, -0.12, 0.0], 
            "A2" : [0.91, 0.27, 0.0], "A2_2" : [0.91, 0.27, 0.0],
            "B1" : [1.64, -0.08, 0.0], "B1_2" : [1.64, -0.08, 0.0], 
            "B2" : [1.64, 0.33, 0.0], "B2_2" : [1.64, 0.33, 0.0],
            "C1" : [2.37, -0.06, 0.0], "C1_2" : [2.37, -0.06, 0.0], 
            "C2" : [2.37, 0.32, 0.0], "C2_2" : [2.37, 0.32, 0.0],
            "R1" : [1.55, -0.9, 0.0], "R2" : [1.55, -1.2, 0.0]
        }

        self.YAW_DICT = {
            "I1" : 4.71, "I2" : 4.71, "I3" : 4.71,
            "O1" : 1.57, "O2" : 1.57, "O3" : 1.5707,
            "P1" : 4.71, "P2" : 4.71, "P3" : 4.71,
            "A1" : 3.14, "A1_2" : 3.14, 
            "A2" : 3.14, "A2_2" : 3.14,
            "B1" : 3.14, "B1_2" : 3.14, 
            "B2" : 3.14, "B2_2" : 3.14, 
            "C1" : 3.14, "C1_2" : 3.14, 
            "C2" : 3.14, "C2_2" : 3.14, 
            "R1" : 1.57, "R2" : 1.57
        }
        self.declare_lists()
        # self.SUB_PATH_POSE_X_LIST = [2.3, 1.7, 0.7, 0.2]
        # self.SUB_PATH_POSE_Y_LIST = [0.7, 0.3, 0.0, -0.5]



        # self.MAIN_PATH_POSE_X_LIST = [2.3, 1.7, 0.7, 0.2]
        # self.MAIN_PATH_POSE_Y_LIST = [0.7, -0.5]
        
        
        
        self.send_robot_status_topics()
        self.get_logger().info("controller is ready")

    def declare_lists(self):
        self.PATH_LIST = [
            [[0.3, -1.1, 0.0],  [0.85, -1.1, 0.0],  [1.15, -1.1, 0.0],  [1.6, -1.1, 0.0]],
            [[0.3, -0.75, 0.0], [0.85, -0.75, 0.0], [1.15, -0.75, 0.0], [1.6, -0.75, 0.0]],
            [[0.3, -0.4, 0.0],  [0.85, -0.4, 0.0],  [1.15, -0.4, 0.0],  [1.6, -0.4, 0.0]],
            [[0.3, 0.0, 0.0],   [0.85, 0.0, 0.0],   [1.15, 0.0, 0.0],   [1.6, 0.0, 0.0]],
            [[0.3, 0.4, 0.0],   [0.85, 0.4, 0.0],   [1.15, 0.4, 0.0],   [1.6, 0.4, 0.0]],
            [[0.3, 0.75, 0.0],  [0.85, 0.75, 0.0],  [1.15, 0.75, 0.0],  [1.6, 0.75, 0.0]],
            [[0.3, 1.1, 0.0],   [0.85, 1.1, 0.0],   [1.15, 1.1, 0.0],   [1.6, 1.1, 0.0]]
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

        self.current_is_passable_list = self.is_passable_list.copy()


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
                self.tasking = True
                self.task_id = req.task_id
                self.current_task_location = req.location
                self.item = req.item
                self.quantity = req.quantity
                
                # pose_list = self.encoding_path(req.location)
                res.success = self.follow_path(self.current_task_location, req.lift)
                self.tasking = False
                self.send_complete_task_topics()

            except Exception as e:
                self.get_logger().error(f"{e}")
                self.tasking = False
                
        else:
            res.success = False

        return res
    
    
    def send_complete_task_topics(self):
        msg = TaskCompletion()

        msg.success = True
        msg.robot_id = ID
        msg.task_id = self.task_id

        for i in range(20):
            self.task_completion_publisher.publish(msg)
            time.sleep(0.1)



    def follow_path(self, pose_name, lift) :
        try:
            self.get_logger().info("path_planning start")
            
            # for pose_name in pose_list:
            target_pose = self.POSE_DICT[pose_name]
            target_yaw = self.YAW_DICT[pose_name]
            

            # self.real_time_stopover_planning(target_pose)

            
            self.get_logger().info(f"goto{pose_name}")
            self.move_pose(target_pose, target_yaw)

            if lift == "Up" : # lift up first place (첫 장소 리프트 업)
                self.get_logger().info("lift up")
                self.service_call_lift(pose_name, "up")
                self.service_call_marker(pose_name, "forward")
                self.service_call_lift(pose_name, "down")
                self.service_call_marker(pose_name, "backward")

            elif lift == "Down" : # lift down last place(마지막 장소 리프트 다운)
                self.get_logger().info("lift down")
                self.service_call_lift(pose_name, "down")
                self.service_call_marker(pose_name, "forward")
                self.service_call_lift(pose_name, "up")
                self.service_call_marker(pose_name, "backward")

            else : # 나머지 장소
                self.current_task_location = pose_name
                self.checking_task_is_out() # 현재 테스크 상태가 OUT이면 대기상태 진입
                

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

        self.move_pose(nearest_point_from_me, [0.0, 0.0, 0.0])

        while current_point_index != nearest_point_from_target_index:
            self.current_is_passable_list
            pass
        

    def search_nearest_point(self,target_pose): # 타겟에 가장 가까운 point 찾기
        nearest_point = [999, 999, 0.0]
        nearest_point_index = [0, 0]
        min_x_vel = 999
        min_y_vel = 999
        for index, vel in enumerate(self.X_LIST):
            distance = abs(target_pose[0] - vel)
            if distance < min_x_vel:
                min_x_vel = distance
                nearest_point[0] = vel
                nearest_point_index[0] = index


        for index, vel in enumerate(self.Y_LIST):
            distance = abs(target_pose[1] - vel)
            if distance < min_y_vel:
                min_y_vel = distance
                nearest_point[1] = vel
                nearest_point_index[1] = index
            
        return nearest_point, nearest_point_index


    def is_passable(self):
        pass

    # def planning_stopover(self, target): # for test
    #     x_min_index = self.find_approximation_to_pose_list(self.MAIN_PATH_POSE_X_LIST, self.my_pose[0])
    #     y_min_index = self.find_approximation_to_pose_list(self.MAIN_PATH_POSE_Y_LIST, self.my_pose[1])
    #     target_x_min_index = self.find_approximation_to_pose_list(self.MAIN_PATH_POSE_X_LIST, target[0])
    #     last_y_min_index = self.find_approximation_to_pose_list(self.SUB_PATH_POSE_Y_LIST, target[1])
        
    
    #     path_poses = [
    #             [
    #                 self.MAIN_PATH_POSE_X_LIST[x_min_index],
    #                 self.MAIN_PATH_POSE_Y_LIST[y_min_index],
    #                 0.0
    #             ],
    #             [
    #                 self.MAIN_PATH_POSE_X_LIST[target_x_min_index],
    #                 self.MAIN_PATH_POSE_Y_LIST[y_min_index],
    #                 0.0
    #             ],
    #             [
    #                 self.MAIN_PATH_POSE_X_LIST[target_x_min_index],
    #                 self.SUB_PATH_POSE_Y_LIST[last_y_min_index],
    #                 0.0
    #             ]
    #         ]
        
    #     return path_poses

    def find_approximation_to_pose_list(self, pose_list, target_vel):
        min_vel = 999
        min_index = 0

        for i, X in enumerate(pose_list):
            vel = abs(target_vel - X)
        
            if vel < min_vel:
                min_vel = vel
                min_index = i

        return min_index



    def encoding_path(self, task) :
        pose_list = task.split("#")
        self.get_logger().info(f"task_list is : {pose_list}")
        
        if "A" in pose_list[0] : # 수거
            self.task_status = "RETURN"

        elif "O" in pose_list[0] : # 출고
            self.task_status = "OUT"


        elif "I" in pose_list[0] : # 입고
            self.task_status = "IN"

        else :
            self.task_status = None
        
        return pose_list

        
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
        self.get_logger().info("before call ")
        self.get_logger().info(f"req.floor : {req.floor}")
        self.get_logger().info(f"req.direction : {req.direction}")
        res = self.lift_client.call_async(req)
        self.wait_lift_res()
        
        # res = self.lift_client.call_async(req)
        # rp.spin_until_future_complete(self, res, timeout_sec=5.0)
        
    def wait_lift_res(self):
        while not self.lift_service_done:
            time.sleep(1)
            # self.get_logger().info("wait lift response")
        self.lift_service_done = False

    def service_call_marker(self, location=None ,direction=None):
        if "_" in location:
            location = location.split("_")[0]

        req = ArucoCommand.Request()
        req.location = location
        req.direction = direction

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
        
        goal_pose = PoseStamped()
        
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = target_pose[0]
        goal_pose.pose.position.y = target_pose[1]
        goal_pose.pose.position.z = target_pose[2]
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        self.nav.goToPose(goal_pose)

        self.nav_distance_feedback()


    def euler_to_quaternion(self, yaw=0, pitch=0, roll=0):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]
    

    def nav_distance_feedback(self) :
        i = 0
        send_data = Float32()
        while not self.nav.isTaskComplete():
            self.check_emergency_status()
            i = i + 1
            feedback = self.nav.getFeedback()

            if feedback and i % 5 == 0 :
                self.get_logger().info("distance remaining: " + "{:.2f}".format(feedback.distance_remaining) + " meters.")
                send_data.data = feedback.distance_remaining
                # self.feedback_publisher.publish(send_data)
                # 추후 distance_remaining 0.10으로 변경할 것
                if (feedback.distance_remaining <= 3.50 and feedback.distance_remaining != 0.0) or Duration.from_msg(feedback.navigation_time) > Duration(seconds=40.0) :
                    time.sleep(2) # 목표지점 인접시 2초후 다음목적지
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
