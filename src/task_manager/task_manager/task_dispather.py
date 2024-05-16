# 로봇 작업 할당
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool

class TaskDispather(Node):
    def __init__(self):
        super().__init__('task_dispather')
        
        self.subscription = self.create_subscription(
            String,       # 메시지 타입
            '/order',      # 토픽 이름
            self.callback,  # 콜백 함수
            10)            # 큐 사이즈
        
        self.publisher = self.create_publisher(Bool, '/task', 10)
        
        self.subscription


    def callback(self, msg):
        bool_value = True if msg.data == 'activate' else False
        # Bool 메시지 생성 및 발행
        self.publisher.publish(Bool(data=bool_value))

def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    task_dispather = TaskDispather()  # 노드 인스턴스 생성
    rclpy.spin(task_dispather)  # 노드가 메시지를 수신하도록 대기
    task_dispather.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
