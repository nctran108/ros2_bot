import rclpy
from rclpy.node import Node
import time
import math
import serial
from threading import Lock

from rclpy.parameter import Parameter
from driver_msgs.msg import MotorVels
from driver_msgs.msg import MotorCommand


class Driver(Node):
    def __init__(self) -> None:
        super().__init__("Driver")

        # setup parameters
        self.declare_parameter('serial_port', value="/dev/ttyACM0")
        self.serial_port = self.get_parameter('serial_port').value

        self.declare_parameter('baud_rate', value=57600)
        self.baud_rate = self.get_parameter('baud_rate').value


        self.declare_parameter('serial_debug', value=False)
        self.debug_serial = self.get_parameter('serial_debug').value
        if self.debug_serial:
            print("Serial debug enabled")

        # setup topics & services
        self.speed_publish = self.create_publisher(MotorVels, 'motor_vels', 10)
        self.subscription = self.create_subscription(
            MotorCommand,
            'motor_command',
            self.motor_command_callback,
            10)

        # Member Variables
        self.left_speed = 0 # for close-loop speed
        self.right_speed = 0 # for close-loop speed
        
        self.mutex = Lock()

        # Open serial comms
        print(f"Connecting to port {self.serial_port} at {self.baud_rate}.")
        self.conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
        print(f"Connected to {self.conn}")

    def send_pwm_motor_command(self, mot_1_pwm, mot_2_pwm):
        self.send_command(f"{int(mot_1_pwm)} {int(mot_2_pwm)}")

    def send_command(self, cmd_string):
        # use mutex for reading motor speed from the motor for close-loop
        try:
            cmd_string += "\r"
            self.conn.write(cmd_string.encode("utf-8"))
        finally:
            if self.debug_serial:
                print('here')
                print("sent: " + cmd_string)
    
    def motor_command_callback(self, motor_command):
        self.send_pwm_motor_command(motor_command.mot_1_req_rad_sec, motor_command.mot_2_req_rad_sec)
        # temp for testing before use close-loop
        spd_msg = MotorVels()
        spd_msg.motor_1_rad_sec = motor_command.mot_1_req_rad_sec
        spd_msg.motor_2_rad_sec = motor_command.mot_2_req_rad_sec
        self.speed_publish.publish(spd_msg)


    def close_conn(self):
        self.conn.close()

def main(args=None):
    rclpy.init(args=args)

    driver_publish = Driver()
    
    rclpy.spin(driver_publish)

    driver_publish.close_conn()
    driver_publish.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()