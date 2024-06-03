import rclpy
from rclpy.node import Node
from task_msgs.srv import AllocateTask

class TaskServer(Node):
    
    
    def __init__(self):
        super().__init__('task_server')
        
        self.robot_id = '213'
        self.topic_name = f'/allocate_task_{self.robot_id}'
        
        self.srv = self.create_service(AllocateTask, self.topic_name, self.allocate_task_callback)

    def allocate_task_callback(self, request, response):
        self.get_logger().info(f'Received task allocation request: {request.task_id}')
        response.success = True  # Success task allocation
        return response
    

def main(args=None):
    rclpy.init(args=args)
    task_server = TaskServer()
    rclpy.spin(task_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
