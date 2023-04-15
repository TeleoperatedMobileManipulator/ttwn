import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from robotarm_interfaces.msg import Sensordeg
import smbus2
import time
from time import *
from adafruit_servokit import ServoKit

bus = smbus2.SMBus(1)
address_multi = 0x70
address = 0x36
address2 = 0x40
channal_2 = 0b00000100
channal_3 = 0b00001000
channal_4 = 0b00010000
global position2_deg
global position3_deg

sumj1 = 0.0

class sensor(Node):

    def __init__(self):
        super().__init__('sensor')
        self.publisher = self.create_publisher(Sensordeg, 'sensordegre_top', 10)
        self.subscriber_ = self.create_subscription(Float32, "degj1_top", self.callback_j1, 10)
        self.get_logger().info("sensor node has been started.")
        #self.publisher2 = self.create_publisher(Float32, 'commu', 10)
        self.create_timer(0.2, self.timer_callback)
        self.create_timer(0.5, self.timer_callback2)


    def callback_j1(self,msg):
        global sumj1
        sumj1 = msg.data
        self.get_logger().info(("deg1 " + str(msg.data)))
        
    def timer_callback(self):
        global position2_deg
        global position3_deg
        global sumj1
        msg = Sensordeg()
        position2_deg, position3_deg = self.read_position()
        self.get_logger().info('position_J2: "%f"' % sumj1) 
        self.get_logger().info('position_J2: "%f"' % position2_deg) 
        self.get_logger().info('position_J3: "%f"' % position3_deg) 
        msg.sendegre1 = round(sumj1,1)
        msg.sendegre2 = round((299-position2_deg),1)
        msg.sendegre3 = round((147.3-position3_deg),1)
        msg.sendegre4 = 0.0
        
        self.publisher.publish(msg)

       
        
    def timer_callback2(self):
        global position2_deg
        global position3_deg
        global position2_ser
        global position3_ser
        position2_ser = 299-position2_deg
        position3_ser = position3_deg - 152
        bus.write_byte(address_multi, channal_2)
        kit = ServoKit(channels=16)
        x = float(190 - (90 + position2_ser - position3_ser))
        if (40<x<180):
            kit.servo[0].angle = x
        else:
            kit.servo[0].angle = 40.0
        sleep(1)
        #kit.servo[3].angle = 90
        #sleep(4)
        
        

    def read_position(self):
        bus.write_byte(address_multi, channal_3)
        position2 = bus.read_word_data(address, 0x0D)
        print(position2)
        bus.write_byte(address_multi, channal_4)
        position3 = bus.read_word_data(address, 0x0D)
        print(position3)
        position2_deg = (position2/4096)*360
        position3_deg = (position3/4096)*360
        return position2_deg, position3_deg



    
def main(args=None):
    rclpy.init(args=args)
    _node = sensor()
    rclpy.spin(_node)
    rclpy.shutdown()

if __name__== '__main__':
    main()