import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TaskReceiver(Node):
    def __init__(self):
        super().__init__('task_receiver')
        self.subscription = self.create_subscription(
            String,       # 메시지 타입
            '/task',      # 토픽 이름
            self.callback,  # 콜백 함수
            10)            # 큐 사이즈
        self.subscription

    def callback(self, msg):
        self.get_logger().info('Received task: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    task_receiver = TaskReceiver()  # 노드 인스턴스 생성
    rclpy.spin(task_receiver)  # 노드가 메시지를 수신하도록 대기
    # 종료 시 정리
    task_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
