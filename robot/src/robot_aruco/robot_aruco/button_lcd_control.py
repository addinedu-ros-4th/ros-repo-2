import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from task_msgs.msg import OutTask
from task_msgs.srv import CompletePicking
from task_msgs.srv import CompletePickingResponse
from robot_aruco.lcd_display import LCDDisplay
from rclpy.executors import MultiThreadedExecutor

class ButtonResponseController(Node):
    def __init__(self, button):
        super().__init__("button_response_controller")
        self.button = button

        # Service Server
        self.lift_server = self.create_service(
            CompletePickingResponse, 
            "/complete_picking_response", 
            self.complete_picking_is_done
        )

        self.subcription_out_task = self.create_subscription(
            OutTask,
            "/out_task",
            self.renew_out_task_data,
            10
        )

        self.subcription_out_task

    def renew_out_task_data(self, data):
        self.button.location = data.location
        self.button.product = data.product
        self.button.count = data.count

    def complete_picking_is_done(self, req, res):
        self.button.response = req
        self.button.picking_is_done = True
        return res

class ButtonLCDControl(Node):
    def __init__(self):
        super().__init__('button_lcd_control')
        
        # Define pin number
        self.button1_pin = 20
        # self.button2_pin = 25
        self.button3_pin = 26
        self.response = CompletePickingResponse.Request()
        # Define LCD 
        self.lcd = LCDDisplay()

        # Set GPIO 
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.button1_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        # GPIO.setup(self.button2_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.button3_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        # Service Client
        self.picking_cli = self.create_client(CompletePicking, '/complete_picking')
        self.req = CompletePicking.Request()
        
        self.location = ""
        self.product = ""
        self.count = 0
        self.picking_is_done = False
        # Publisher for emergency stop
        self.emergency_stop_pub = self.create_publisher(Empty, '/emergency_stop', 10)
        
        # Button event
        GPIO.add_event_detect(self.button1_pin, GPIO.RISING, callback=self.button1_callback, bouncetime=300)
        # GPIO.add_event_detect(self.button2_pin, GPIO.RISING, callback=self.button2_callback, bouncetime=300)
        GPIO.add_event_detect(self.button3_pin, GPIO.RISING, callback=self.button3_callback, bouncetime=300)


    def send_picking_request(self):
        self.req.task_done = True
        self.get_logger().info('next button is clicked')
        self.future = self.picking_cli.call_async(self.req)
        self.wait_complete_picking()

        return self.future.result()
        

    def wait_complete_picking(self):
        while not self.picking_is_done:
            time.sleep(1)
        self.picking_is_done = False

    # Emergency case
    def button1_callback(self, channel):
        self.get_logger().info('Emergency Stop button pressed!')
        self.emergency_stop_pub.publish(Empty())


    # Outbound: move to next place
    def button3_callback(self, channel):
        # response = self.send_picking_request()

        self.lcd.send_command(0x01)
        self.lcd.write(4, 0, f"OB : {self.location}")
        if self.count > 0:
            self.lcd.write(2, 1, f"{self.product} :{self.count} ")


def main() :
    rclpy.init()
    btn = ButtonLCDControl()
    rescon = ButtonResponseController(button = btn)


    excutor = MultiThreadedExecutor()

    excutor.add_node(btn)
    excutor.add_node(rescon)
    
    try : 
        excutor.spin()
    finally : 
        excutor.shutdown()
        btn.destroy_node()
        rescon.destroy_node()
        rclpy.shutdown()
    # node = ButtonLCDControl()
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()



if __name__ == '__main__' :
    main()