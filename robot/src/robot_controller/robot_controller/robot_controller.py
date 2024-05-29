import rclpy as rp
import os
from rclpy.node import Node
from task_msgs.srv import AllocateTask
from task_msgs.srv import ArucoCommand
from task_msgs.srv import StepControl
from std_msgs.msg import Bool
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
        super().__init__("robot_subscriber")
        self.controller = controller
        self.subcription_amclpose = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.current_pose,
            10
        )
        

    def current_pose(self, data):
        self.controller.my_pose[0] = data.pose.pose.position.x
        self.controller.my_pose[1] = data.pose.pose.position.y

ID = os.getenv('ROS_DOMAIN_ID', 'Not set')
class RobotController(Node) : 
    def __init__(self) : 
        super().__init__("robot_controller")
        self.get_logger().info("start robot_controller")
        self.nav = BasicNavigator()
        
        self.tasking = False
        self.task_status = None
        self.my_pose = [0.0, 0.0, 0.0]
        self.server = self.create_service(AllocateTask, f"/task_{ID}", self.task_callback)
        self.publisher = self.create_publisher(
            Bool,
            "/task_success",
            10
        )

        self.feedback_publisher = self.create_publisher(
            Float32,
            "/feedback",
            10
        )
        # self.subcription_amclpose = self.create_subscription(
        #     PoseWithCovarianceStamped,
        #     "/amcl_pose",
        #     self.current_pose,
        #     10
        # )

        # self.wait_client = self.create_client(

        # )

        self.lift_client = self.create_client(
            StepControl, "/step_control"
        )

        self.arucomarker_client = self.create_client(
            ArucoCommand, "/aruco_control"
        )

        # pose x, y, z, rad x, y, z, w 
        # 추후 json으로 변경 필요
        self.POSE_DICT = {
            "I1" : [2.4367, -0.5634, 0.0], "I2" : [2.0961, -0.5972, 0.0], "I3" : [1.7499, -0.6143, 0.0],
            "O1" : [0.1658, 0.8952, 0.0], "O2" : [0.4006, 0.8573, 0.0], "O3" : [0.7368, 0.9275, 0.0],
            "P1" : [0.2368, -0.6169, 0.0], "P2" : [0.5033, -0.6484, 0.0], "P3" : [0.7635, -0.6800, 0.0],
            "A1" : [0.2791, 0.8958, 0.0], "A2" : [-0.0731, 0.8900, 0.0],
            "B1" : [0.2791, 0.8958, 0.0], "B2" : [-0.0731, 0.8900, 0.0],
            "C1" : [0.2791, 0.8958, 0.0], "C2" : [-0.0731, 0.8900, 0.0],
            "R1" : [2.2625, 0.8958, 0.0], "R2" : [2.027, 0.8900, 0.0]
            }
        
        self.SUB_PATH_POSE_X_LIST = [2.3, 1.7, 0.7, 0.2]
        self.SUB_PATH_POSE_Y_LIST = [0.3, 0.0]



        self.MAIN_PATH_POSE_X_LIST = [2.3, 1.7, 0.7, 0.2]
        self.MAIN_PATH_POSE_Y_LIST = [0.7, -0.5]
        
        # self.subscription
        

        self.get_logger().info("waiting client")


    def euler_to_quaternion(self, yaw=0, pitch=0, roll=0):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]


    # def current_pose(self, data):
    #     self.my_pose[0] = data.pose.pose.position.x
    #     self.my_pose[1] = data.pose.pose.position.y
        

    def planning_stopover(self, target):
        y_min_vel = 999
        y_min_index = 0
        target_y_min_vel = 999
        target_y_min_index = 0

        for i, Y in enumerate(self.MAIN_PATH_POSE_Y_LIST):
            y_vel = abs(self.my_pose[1] - Y)
            target_y_vel = abs(target[1] - Y)
            if y_vel < y_min_vel:
                y_min_vel = y_vel
                y_min_index = i

            if target_y_vel < target_y_min_vel:
                target_y_min_vel = target_y_vel
                target_y_min_index = i

        x_min_vel = 999
        x_min_index = 0
        target_x_min_vel = 999
        target_x_min_index = 0

        for i, X in enumerate(self.MAIN_PATH_POSE_X_LIST):
            x_vel = abs(self.my_pose[0] - X)
            target_x_vel = abs(target[0] - X)

            if x_vel < x_min_vel:
                x_min_vel = x_vel
                x_min_index = i

            if target_x_vel < target_x_min_vel:
                target_x_min_vel = target_x_vel
                target_x_min_index = i
    
        path_poses = [
            [self.MAIN_PATH_POSE_X_LIST[x_min_index],
             self.MAIN_PATH_POSE_Y_LIST[y_min_index],
             0.0],
             [self.MAIN_PATH_POSE_X_LIST[target_x_min_index],
              self.MAIN_PATH_POSE_Y_LIST[y_min_index],
              0.0],
              [self.MAIN_PATH_POSE_X_LIST[target_x_min_index],
              self.MAIN_PATH_POSE_Y_LIST[target_y_min_index],
              0.0]
             ]
        
        return path_poses

        

    def task_callback(self, req, res) :
        self.get_logger().info("task_list:" + req.location)
        if not self.tasking:
            try:
                self.tasking = True
                pose_list = self.encoding_path(req.location)
                res.success = self.follow_path(pose_list)
                self.tasking = False
            except:
                self.tasking = False
                
        else:
            res.success = False

        return res
    

    def nav_distance_feedback(self) :
        i = 0
        send_data = Float32()
        while not self.nav.isTaskComplete():
            i = i + 1
            feedback = self.nav.getFeedback()
            if feedback and i % 5 == 0 :
                self.get_logger().info("distance remaining: " + "{:.2f}".format(feedback.distance_remaining) + " meters.")
                send_data.data = feedback.distance_remaining
                self.feedback_publisher.publish(send_data)

                if (feedback.distance_remaining <= 0.35 and feedback.distance_remaining != 0.0) or Duration.from_msg(feedback.navigation_time) > Duration(seconds=40.0) :
                    self.nav.cancelTask()
                    self.get_logger().info("cancel Task")
                    return


    
    def service_call_marker(self, location=None ,direction=None):
        req = ArucoCommand.Request()
        req.location = location
        req.direction = direction
        res = self.arucomarker_client.call(req)
    

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


        # 
        



    def follow_path(self, pose_list) :
        # try:
        self.get_logger().info("path_planning start")
        

        for pose_name in pose_list:
            target_pose = self.POSE_DICT[pose_name]
            path_poses = self.planning_stopover(target_pose)
            self.get_logger().info(f"my_pose : {self.my_pose}")
            self.get_logger().info(f"stopover : {path_poses}")
            for pose in path_poses: # stopover (경유지 이동)
                self.get_logger().info("goto stopover")
                self.move_pose(pose, 0.0)
            
            self.get_logger().info(f"goto{pose_name}")
            self.move_pose(target_pose, 0.0)

            if pose_name == pose_list[0] : # lift up first place (첫 장소 리프트 업)
                self.get_logger().info("lift up")
                # self.service_call_marker(pose_name, "forward")
                self.service_call_lift_up(pose_name)

            elif pose_name == pose_list[-1] : # lift down last place(마지막 장소 리프트 다운)
                self.get_logger().info("lift down")
                # self.service_call_marker(pose_name, "backward")
                self.service_call_lift_down()
            

            self.tasking = False
            self.get_logger().info("move end")

        return True
            
        # except Exception as e:
        #     self.get_logger().error(f"{e} in follow_path")
        #     self.tasking = False
        #     return False

    def service_call_lift_up(self, pose_name) :
        pass

    def service_call_lift_down(self):
        pass

    def move_pose(self, target_pose, roll) :
        q = self.euler_to_quaternion(roll=roll)
        self.get_logger().info(f"pose : {target_pose} roll : {roll}")
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


    # task_subscriber = RobotController()
    # rp.spin(task_subscriber) # while 1 :
    
    # task_subscriber.destroy_node()
    # rp.shutdown()
    


if __name__ == "__main__" : 
    main()
