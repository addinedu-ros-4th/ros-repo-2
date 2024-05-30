import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from task_msgs.srv import CompletePicking
from lcd_display import LCDDisplay


class ButtonLCDControl(Node):
    def __init__(self):
        super().__init__('Button_Lcd_Node')
        
        # Define pin number
        self.button1_pin = 20
        # self.button2_pin = 25
        self.button3_pin = 26

        # Define LCD 
        self.lcd = LCDDisplay()

        # Set GPIO 
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.button1_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        # GPIO.setup(self.button2_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.button3_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        # Service Client
        self.picking_cli = self.create_client(CompletePicking, 'complete_picking')
        self.req = CompletePicking.Request()
        
        # Publisher for emergency stop
        self.emergency_stop_pub = self.create_publisher(Empty, 'emergency_stop', 10)
        
        # Button event
        GPIO.add_event_detect(self.button1_pin, GPIO.RISING, callback=self.button1_callback, bouncetime=300)
        # GPIO.add_event_detect(self.button2_pin, GPIO.RISING, callback=self.button2_callback, bouncetime=300)
        GPIO.add_event_detect(self.button3_pin, GPIO.RISING, callback=self.button3_callback, bouncetime=300)


    def send_picking_request(self):
        self.req.task_done = True
        self.future = self.picking_cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


    # Emergency case
    def button1_callback(self):
        self.get_logger().info('Emergency Stop button pressed!')
        self.emergency_stop_pub.publish(Empty())


    # Outbound: move to next place
    def button3_callback(self):
        response = self.send_picking_request()
        location = response.location
        product = response.product
        count = response.count

        self.lcd.send_command(0x01)
        self.lcd.write(4, 0, f"OB :  {location}")
        self.lcd.write(0, 1, f"{product} : {count}")


def main() :
    rclpy.init()
    node = ButtonLCDControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()