from typing import List
import rclpy
from rclpy.context import Context
from rclpy.node import Node
import time
from tkinter import *
import math

from driver_msgs.msg import MotorCommand
from rclpy.parameter import Parameter

class Command(Node):
    def __init__(self) -> None:
        super().__init__("Command")
        self.publisher = self.create_publisher(MotorCommand, 'motor_command', 10)

        self.m1 = 0
        self.m2 = 0
        self.msg = MotorCommand()
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period,self.publish_speed)

    def show_value(self):
        print (self.m1, self.m2)
    
    def send_motor_once(self):
        self.update_scale_limits()
        self.msg.mot_1_req_rad_sec = float(self.m1)
        self.msg.mot_2_req_rad_sec = float(self.m2)
        self.publisher.publish(self.msg)

    def stop_motor(self):
        self.msg.mot_1_req_rad_sec = 0.0
        self.msg.mot_2_req_rad_sec = 0.0
        self.publisher.publish(self.msg)

    def update_scale_limits(self):
        if self.m1 < -255:
            self.m1 = -255
        elif self.m1 > 255: 
            self.m1 = 255
        if self.m2 < -255:
            self.m2 = -255
        elif self.m2 > 255: 
            self.m2 = 255

    def publish_speed(self):
        msg = input()
        if msg == "stop":
            self.stop_motor()
        else:
            msgs = msg.split(" ")
            if (len(msgs) == 2):
                if (msgs[0] == 'l'):
                    self.m1 = float(msgs[1])
                elif (msgs[0] == 'r'):
                    self.m2 = float(msgs[1])
                else:
                    self.m1 = float(msgs[0])
                    self.m2 = float(msgs[1])
            else:
                print("invalid input")
            self.send_motor_once()

def main(args=None):
    rclpy.init(args=args)

    motor_command = Command()

    rclpy.spin(motor_command)
    
    motor_command.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


