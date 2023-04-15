#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
from time import sleep
from robotarm_interfaces.msg import Sensordeg

global degree
servopin = 18
P = None

class servo(Node):
    def __init__(self):
        global servopin
        global P
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(servopin, GPIO.OUT)
        P = GPIO.PWM(servopin, 50)
        P.start(0)
        #factory = PiGPIOFactory()
        #servo =AngularServo(18, min_pulse_width=0.0005, max_pulse_width=0.0025, min_angle=93, max_angle=-93)
        #servo.angle = 0
        super().__init__("servo")
        self.subscription = self.create_subscription(Sensordeg, 'sensordegre_top', self.callback_sensor, 10)
        self.get_logger().info("servo node has been started.")
        self.create_timer(1.0, self.timer_callback)

        
    
    def callback_sensor(self, msg):
        global sensor2
        global sensor3
        sensor2 = msg.sendegre2
        sensor3 = msg.sendegre3
        self.get_logger().info('position_J2: "%f"' % sensor2) 
        self.get_logger().info('position_J3: "%f"' % sensor3)
    
    def timer_callback(self):
        global P
        global degree 
        degree = input('Enter: ')
        degree2 = float(degree)
        P.ChangeDutyCycle(degree2)
        sleep(0.5)
        
    
    

def main(args=None):
    rclpy.init(args=args)
    node = servo()
    rclpy.spin(node)
    rclpy.shutdown()

    global P
    P.stop()
    GPIO.cleanup()


if __name__ == "__main__":
    main()
