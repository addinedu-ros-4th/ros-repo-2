import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class OrderReceiver(Node):
    def __init__(self):
        super().__init__('order_receiver')
        self.subscription = self.create_subscription(
            String,       # 메시지 타입
            '/order',      # 토픽 이름
            self.callback,  # 콜백 함수
            10)            # 큐 사이즈
        self.subscription

    def callback(self, msg):
        order_data = json.loads(msg.data)
        self.get_logger().info('Ordered task: "%s"' % order_data)

def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    task_receiver = OrderReceiver()  # 노드 인스턴스 생성
    rclpy.spin(task_receiver)  # 노드가 메시지를 수신하도록 대기
    # 종료 시 정리
    task_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
