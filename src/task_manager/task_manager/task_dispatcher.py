# 로봇 작업 할당
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from task_msgs.msg import Task, RobotState, Inbound
import json

class TaskDispatcher(Node):
    def __init__(self):
        super().__init__('task_dispatcher')
        
        self.robot_ready = False
        self.order_list = []
        self.inbound_list = []
        
        # Robot state
        self.state_subscription = self.create_subscription(
            RobotState,
            '/robot_state',
            self.state_callback,
            10
        )
        
        # Order
        self.order_subscription = self.create_subscription(
            String,       
            '/order',      
            self.order_callback,  
            10
        )
        
        # InBound
        self.inbound_subscription = self.create_subscription(
            String,       
            '/inbound',      
            self.inbound_callback,  
            10
        )
        
        # Task
        self.task_publisher = self.create_publisher(Task, '/task', 10)
        self.timer = self.create_timer(1.0, self.process_orders)
    
    
    # Robot state 
    def state_callback(self, msg):
        self.robot_ready = msg.is_ready

    
    # Add order in order list
    def order_callback(self, msg):
        try:
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

        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error: {e}")
        except KeyError as e:
            self.get_logger().error(f"Missing key in order_info: {e}")
        except TypeError as e:
            self.get_logger().error(f"Type error in order_info: {e}")
    

    def process_single_order(self, single_order):
        try:
            user_id = single_order['user_id']
            items = single_order['items']
            quantities = single_order['quantities']

            for item, quantity in zip(items, quantities):
                if item == 'cola':
                    location = 'A1'
                elif item == 'water':
                    location = 'A2'
                elif item == 'ramen':
                    location = 'A3'
                else:
                    location = 'X'

                self.order_list.append({
                    'user_id': user_id,
                    'item': item,
                    'quantity': quantity,
                    'location': location
                })

            self.get_logger().info(f"Order added: user_id={user_id}, items={items}, quantities={quantities}")
            
        except KeyError as e:
            self.get_logger().error(f"Missing key in single order: {e}")
        except TypeError as e:
            self.get_logger().error(f"Type error in single order: {e}")


    # Exist order_list and robot's task success, assignment next task
    def process_orders(self):
        if self.robot_ready:
            if self.order_list:
                order = self.order_list.pop(0)
                task_msg = Task()
                task_msg.location = order['location']
                task_msg.task_name = 'OutBound'
                task_msg.quantity = order['quantity']
                task_msg.item_name = order['item']
                
                self.task_publisher.publish(task_msg)
                self.get_logger().info(f"Published task: {task_msg}")
            elif self.inbound_list:
                inBound_msg = Inbound()
                self.task_publisher.publish(inBound_msg)
                
    def inbound_callback(self):
        # 데이터 가져와서 inBound list에 넣기
        pass


def main(args=None):
    rclpy.init(args=args)
    task_dispatcher = TaskDispatcher()
    rclpy.spin(task_dispatcher)
    task_dispatcher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()