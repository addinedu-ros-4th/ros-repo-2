import rclpy as rp
import os
from rclpy.node import Node
from task_msgs.srv import AllocateTask
from std_msgs.msg import Bool
from std_msgs.msg import Float32

from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import TaskResult
from geometry_msgs.msg import PoseStamped
ID = os.getenv('ROS_DOMAIN_ID', 'Not set')
class TaskSubScriber(Node) : 
    def __init__(self) : 
        super().__init__("task_subscriber")
        self.get_logger().info("start task_subscriber")
        self.nav = BasicNavigator()
        self.tasking = False
        # self.nav.waitUntilNav2Active()
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
            res.task_success = self.move_pose(req.task_list)
            self.tasking = True
        
        return res

    def nav_distance_feedback(self):
        i = 0
        send_data = Float32()
        while not self.nav.isTaskComplete():
            i = i + 1
            feedback = self.nav.getFeedback()
            if feedback and i % 5 == 0 :
                self.get_logger().info("distance remaining: " + "{:.2f}".format(feedback.distance_remaining) + " meters.")
                send_data.data = feedback.distance_remaining
                self.feedback_publisher.publish(send_data)

                if feedback.distance_remaining <= 0.05 or Duration.from_msg(feedback.navigation_time) > Duration(seconds=40.0):
                    self.nav.cancelTask()
                    self.get_logger().info("cancel Task")
                    return
    


    def move_pose(self, task):
        pose_list = task.split("#")
        try:
            self.get_logger().info("move start")
            if "A" in pose_list[0]: # 수거
                pass
            if "O" in pose_list[0]: # 출고
                pass
            if "I" in pose_list[0]: # 입고
                pass

            for pose_name in pose_list:
                
                target_pose = self.POSE_DICT[pose_name]
                
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


                self.tasking = False
                self.get_logger().info("move end")
                return True
            
        except Exception as e:
            self.get_logger().error(f"{e} in move pose")
            self.tasking = False
            return False



def main(args = None) : 
    rp.init(args=args)
    
    task_subscriber = TaskSubScriber()
    rp.spin(task_subscriber) # while 1 :
    
    task_subscriber.destroy_node()
    rp.shutdown()
    


if __name__ == "__main__" : 
    main()
