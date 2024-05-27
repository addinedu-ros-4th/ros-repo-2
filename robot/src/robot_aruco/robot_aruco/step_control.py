import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
from task_msgs.srv import StepControl

class RasGPIOController:
    def __init__(self, step_pins):
        # pin 번호 설정
        self.step_pins = step_pins
        
        # GPIO 설정
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        for pin in self.step_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, False)

        # floor 설정 
        self.current_step = 0
        self.stepfloors = {"1place": 0, "1lift": -3000, "2place": -6000, "2lift": -7000}

    def step_control(self, floor):
        desired_step  = self.stepfloors[floor]
        step_diff = desired_step - self.current_step 
         
        StepCount = 4

        Seq = [[1, 0, 0, 0],
               [0, 1, 0, 0],
               [0, 0, 1, 0],
               [0, 0, 0, 1]]
        
        StepCounter = 0
        direction = 1 if step_diff > 0 else -1
        step_diff = abs(step_diff)
        WaitTime = 0.002

        for _ in range(step_diff):
            for pin in range(0, 4):
                xpin = self.step_pins[pin]
                if Seq[StepCounter][pin] != 0:
                    GPIO.output(xpin, True)
                else:
                    GPIO.output(xpin, False)
            StepCounter += direction
            if StepCounter >= StepCount:
                StepCounter = 0
            if StepCounter < 0:
                StepCounter = StepCount - 1
            time.sleep(WaitTime)
        
        self.current_step = desired_step


class RobotStepControl(Node):
    def __init__(self):
        self.rascontroller = RasGPIOController([17, 18, 22, 23])
        self.server = self.create_service(StepControl, '/step_control', self.handle_forkarm)
    

    def handle_forkarm(self, request, response):
        self.floor = request.floor 
        self.direction = request.direction 
        
        if self.direction == 'up':
            self.rascontroller.step_control(f'{self.floor}lift')
            response = True

        elif self.direction == 'down':
            self.rascontroller.step_control(f'{self.floor}place')
            response = True

        else: 
            response = False
            return response

        return response