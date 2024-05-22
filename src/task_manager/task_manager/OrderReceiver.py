import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from task_msgs.msg import TaskList, Task
import json
import uuid

class OrderReceiver(Node):
    def __init__(self):
        super().__init__('order_receiver')
        
        self.subscription = self.create_subscription(String, '/order', self.order_callback, 10)
        self.task_list_publisher = self.create_publisher(TaskList, '/task_list', 10)

        self.order_list = []
        
        
    def order_callback(self, msg):
        order_info = json.loads(msg.data)  # JSON 문자열을 파싱하여 리스트로 변환

        # order_info가 리스트인지 확인
        if isinstance(order_info, list):
            for single_order in order_info:
                if isinstance(single_order, dict):
                    self.process_single_order(single_order)
                else:
                    self.get_logger().warn(f"Invalid order format: {single_order}")
        else:
            self.get_logger().warn("order_info is not a list")


    def process_single_order(self, single_order):
        try:
            user_id = single_order['user_id']
            items = single_order['items']
            quantities = single_order['quantities']

            tasks = []
            for item, quantity in zip(items, quantities):
                if item == 'cola':
                    location = 'A'
                elif item == 'water':
                    location = 'B'
                elif item == 'ramen':
                    location = 'C'
                else:
                    location = 'X'

                self.order_list.append({
                    'user_id': user_id,
                    'items': items,
                    'quantities': quantities
                })
                
                task = Task()
                task.task_id = str(uuid.uuid4())
                task.task_type = "Outbound"
                task.priority = 3
                task.item = items
                task.quantity = quantities
                task.location = location
                
                tasks.append(task)

            self.get_logger().info(f"Order added: user_id={user_id}, items={items}, quantities={quantities}")

            # 작업 리스트를 게시
            self.publish_task_list(tasks)
            
        except KeyError as e:
            self.get_logger().error(f"Missing key in single order: {e}")
        except TypeError as e:
            self.get_logger().error(f"Type error in single order: {e}")
            
            
    def publish_task_list(self, tasks):
        task_list = TaskList()
        task_list.tasks.extend(tasks)
        self.task_list_publisher.publish(task_list)
        

def main():
    rclpy.init()
    node = OrderReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
