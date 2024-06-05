import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from task_msgs.msg import TaskList, Task
import json
import uuid

class OrderReceiver(Node):
    def __init__(self):
        super().__init__('order_receiver')
        
        # Subscription settings for each topic
        self.order_subscription = self.create_subscription(String, '/outbound', self.create_callback('/outbound'), 10)
        self.inbound_subscription = self.create_subscription(String, '/inbound', self.create_callback('/inbound'), 10)
        self.task_list_publisher = self.create_publisher(TaskList, '/task_list', 10)
        
        
    def order_callback(self, topic_name, msg):
        self.get_logger().info(f'Received message on {topic_name}: {msg.data}')
        try:
            order_info = json.loads(msg.data)
            self.get_logger().info(f'Parsed order info: {order_info}')
            order_type = "outbound" if topic_name == "/outbound" else "inbound"
            print(order_type)
            
            if isinstance(order_info, dict):
                self.process_single_order(order_info, order_type)    
            elif isinstance(order_info, list):
                for single_order in order_info:
                    if isinstance(single_order, dict):
                        self.process_single_order(single_order, order_type)
                    else:
                        self.get_logger().warn(f"Invalid order format: {single_order}")
            else:
                self.get_logger().warn("order_info is not a list")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error: {str(e)}")
        except TypeError as e:
            self.get_logger().error(f"Type error: {str(e)}")
    

    def create_callback(self, topic_name):
        def callback(msg):
            self.order_callback(topic_name, msg)
        return callback
    
    
    def process_single_order(self, single_order, order_type):
        self.get_logger().info(f'Processing single order: {single_order}')
        try:
            if order_type == 'inbound':
                bundle_id = single_order['inbound_id']
                inbound_zone = single_order.get('inbound_zone', 'default_zone')
                item_names = [single_order['item_name']]
                quantities = [single_order['quantities']]
            else:
                bundle_id = single_order['user_id']
                item_names = single_order['item_name']
                quantities = single_order['quantities']
                
            tasks = []
            
            for item_name, quantity in zip(item_names, quantities):
                task = Task(
                        task_id=str(uuid.uuid4()),
                        task_type="IB" if order_type == "inbound" else "OB",
                        priority=3 if order_type == "inbound" else 1,
                        bundle_id=str(bundle_id),
                        item=item_name,
                        quantity=quantity,
                        location=inbound_zone if order_type == "inbound" else self.get_location_for_item(item_name),
                        lift="X" if order_type == "inbound" else "Up"
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
        item_locations = {
            'Coke': 'A1',
            'Sprite': 'A2',
            'Chapagetti': 'B1',
            'Buldak': 'B2',
            'Robot Vacuum': 'C1',
            'Coffee Pot': 'C2'
        }
        return item_locations.get(item, 'X')      
            
            
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
