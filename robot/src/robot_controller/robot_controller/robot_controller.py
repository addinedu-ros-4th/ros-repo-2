import rclpy as rp
import os
from rclpy.node import Node
from task_msgs.srv import AllocateTask
from task_msgs.srv import ArucoCommand
from std_msgs.msg import Bool
from std_msgs.msg import Float32

from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import TaskResult
from geometry_msgs.msg import PoseStamped
ID = os.getenv('ROS_DOMAIN_ID', 'Not set')
class RobotController(Node) : 
    def __init__(self) : 
        super().__init__("robot_controller")
        self.get_logger().info("start robot_controller")
        self.nav = BasicNavigator()
        
        self.tasking = False
        self.task_status = None

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
        self.wait_client = self.create_client(

        )
        self.lift_client = self.create_client(

        )
        self.arucomarker_client = self.create_client(
            ArucoCommand, "/aruco_control"
        )

        # pose x, y, z, rad x, y, z, w 
        # 추후 json으로 변경 필요
        self.POSE_DICT = {
            "I1" : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "I2" : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "I3" : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "O1" : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "O2" : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "O3" : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "P1" : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "P2" : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "P3" : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "A1" : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "A2" : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "A3" : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "A4" : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "A5" : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "A6" : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "R1" : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "R2" : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            }
        # 추후 추가예정
        self.PATH_DICT = {
            
        }
        # self.subscription
        

        self.get_logger().info("waiting client")

        

    def task_callback(self, req, res) :
        self.get_logger().info("task_list:" + req.task_list)
        if not self.tasking:
            res.task_success = self.planning_path(req.task_list)
            self.tasking = True
        
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

                if feedback.distance_remaining <= 0.05 or Duration.from_msg(feedback.navigation_time) > Duration(seconds=40.0) :
                    self.nav.cancelTask()
                    self.get_logger().info("cancel Task")
                    return
    
    def service_call_marker(self, location=None ,direction=None):
        self.arucomarker_client.location = location
        self.arucomarker_client.direction = direction
        self.arucomarker_client.call()
                
    def planning_path(self, task) :
        pose_list = task.split("#")

        try:
            self.get_logger().info("path_planning start")
            if "A" in pose_list[0] : # 수거
                self.task_status = "RETURN"

            if "O" in pose_list[0] : # 출고
                self.task_status = "OUT"

            if "I" in pose_list[0] : # 입고
                self.task_status = "IN"

            for pose_name in pose_list:
                
                target_pose = self.POSE_DICT[pose_name]
                
                
                self.move_pose(target_pose)

                if pose_name == pose_list[0] : # 첫 장소 리프트 업
                    self.get_logger().info("lift up")
                    self.service_call_marker(pose_name, "forward")
                    self.service_call_lift_up(pose_name)

                elif pose_name == pose_list[-1] : # 마지막 장소 리프트 다운
                    self.get_logger().info("lift down")
                    self.service_call_lift_down()
                

                self.tasking = False
                self.get_logger().info("move end")

            return True
            
        except Exception as e:
            self.get_logger().error(f"{e} in planning_path")
            self.tasking = False
            return False

    def service_call_lift_up(self, pose_name) :
        pass

    def service_call_lift_down(self):
        pass

    def move_pose(self, target_pose) :
                
        # self.get_logger().info("test_msg")
        goal_pose = PoseStamped()
        
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = target_pose[0]
        goal_pose.pose.position.y = target_pose[1]
        goal_pose.pose.position.z = target_pose[2]
        goal_pose.pose.orientation.x = target_pose[3]
        goal_pose.pose.orientation.y = target_pose[4]
        goal_pose.pose.orientation.z = target_pose[5]
        goal_pose.pose.orientation.w = target_pose[6]

        self.nav.goToPose(goal_pose)

        self.nav_distance_feedback()



def main(args = None) : 
    rp.init(args=args)
    
    task_subscriber = RobotController()
    rp.spin(task_subscriber) # while 1 :
    
    task_subscriber.destroy_node()
    rp.shutdown()
    


if __name__ == "__main__" : 
    main()
