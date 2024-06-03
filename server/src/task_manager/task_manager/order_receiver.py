import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from task_msgs.msg import TaskList, Task
import json
import uuid

class OrderReceiver(Node):
    def __init__(self):
        super().__init__('order_receiver')
        
        self.order_list = []
        
        self.subscription = self.create_subscription(String, '/order', self.order_callback, 10)
        self.task_list_publisher = self.create_publisher(TaskList, '/task_list', 10)
        
        
    def order_callback(self, msg):
        try:
            # msg.data를 직접 리스트로 파싱
            order_info = json.loads(msg.data)
            
            if isinstance(order_info, list):
                for single_order in order_info:
                    if isinstance(single_order, dict):
                        self.process_single_order(single_order)
                    else:
                        self.get_logger().warn(f"Invalid order format: {single_order}")
            else:
                self.get_logger().warn("order_info is not a list")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error: {str(e)}")
        except TypeError as e:
            self.get_logger().error(f"Type error: {str(e)}")


    def process_single_order(self, single_order):
        try:
            bundle_id = single_order['user_id']
            items = single_order['item_name']
            quantities = single_order['quantities']

            tasks = []
            for item, quantity in zip(items, quantities):

                self.order_list.append({
                    'user_id': bundle_id,
                    'items': items,
                    'quantities': quantities
                })
                
                task = Task(
                    task_id=str(uuid.uuid4()),
                    task_type="OB",              # Assuming outbound for example
                    priority=3,                  # Example priority
                    bundle_id=str(bundle_id),    # Use bundle_id
                    item=item,
                    quantity=quantity,
                    location=self.get_location_for_item(item),
                    lift = "X"
                )
            
                tasks.append(task)

            # 작업 리스트를 게시
            self.publish_task_list(tasks)
            self.get_logger().info(f"Success send task list {tasks}")
    
            
        except KeyError as e:
            self.get_logger().error(f"Missing key in single order: {e}")
        except TypeError as e:
            self.get_logger().error(f"Type error in single order: {e}")
    
    
    def get_location_for_item(self, item):
        # Determine the location for a given item
        if item == 'Coke':
            return 'A1'
        elif item == 'Sprite':
            return 'A2'
        elif item == 'Chapagetti':
            return 'B1'
        elif item == 'Buldak':
            return 'B2'
        elif item == 'Robot Vacuum':
            return 'C1'
        elif item == 'Coffee Pot':
            return 'C2'
        else:
            return 'X'        
            
            
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
