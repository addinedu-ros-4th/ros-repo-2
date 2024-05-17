# 로봇 작업 할당
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from task_msgs.msg import Order, Task
import json

class TaskDispather(Node):
    def __init__(self):
        super().__init__('task_dispather')
        
        self.subscription = self.create_subscription(
            String,       # 메시지 타입
            '/order',      # 토픽 이름
            self.order_callback,  # 콜백 함수
            10)            # 큐 사이즈
        

        self.task_publisher = self.create_publisher(Task, '/task', 10)
        
        self.order_list = []
        
        # 주기적으로 로봇 상태를 확인하고 Task 발행
        self.timer = self.create_timer(2.0, self.process_orders)


    # order list에 저장 (location 추가)
    def order_callback(self, msg):
        order_info = json.loads(msg.data)  # JSON 문자열을 파이썬 딕셔너리로 변환

        user_id = order_info['user_id']  # user_id 추출
        items = order_info['items']
        quantities = order_info['quantities']

        for item, quantity in zip(items, quantities):
            if item == 'cola':
                location = 'A1'
            elif item == 'water':
                location = 'A2'
            elif item == 'ramen':
                location = 'A3'
            else :
                location = 'X'
            
            self.order_list.append({
            'user_id': user_id,
            'item': item,
            'quantity': quantity,
            'location': location
            })
             

        
    def process_orders(self):
        # 로봇이 대기 상태일 때 작업 처리
        if self.is_robot_ready() and self.order_list:
            order = self.order_list.pop(0)
            task_msg = Task()
            task_msg.location = order['location']
            task_msg.task_name = 'OutBound'
            task_msg.quantity = order['quantity']
            task_msg.item_name = order['item']
            
            self.task_publisher.publish(task_msg)
            self.get_logger().info(f"Published task: {task_msg}")
    
    
    def is_robot_ready(self):
        # 로봇의 상태를 확인
        # robot 상태 받기
        # self.pose_subscription = self.create_subscription(
            
        # )
        return True


def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    task_dispather = TaskDispather()  # 노드 인스턴스 생성
    rclpy.spin(task_dispather)  # 노드가 메시지를 수신하도록 대기
    task_dispather.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
