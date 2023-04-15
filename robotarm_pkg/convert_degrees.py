#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robotarm_interfaces.msg import Positiondegrees
from robotarm_interfaces.msg import Inversedegrees
from robotarm_interfaces.msg import Forwarddegrees

#initial distance 
offset_J1 = 0.0
offset_J2 = 0.0
offset_J3 = 0.0
offset_J4 = 0.0

#interval distance for move
interval_J1 = 1.0
interval_J2 = 2.0
interval_J3 = 1.8
interval_J4 = 1

#share variable
position_J1 = 0.0
position_J2 = 0.0
position_J3 = 0.0
position_J4 = 0.0

inverse_J1 =0.0
inverse_J2 =0.0
inverse_J3 =0.0
inverse_J4 =0.0

plusdeg_J1 = 0.0
plusdeg_J2 = 0.0
plusdeg_J3 = 0.0
plusdeg_J4 = 0.0


class convert_degrees(Node):
    def __init__(self):
        super().__init__("convert_degrees")
        self.subscription = self.create_subscription(String, "webint_top", self.callback_robot, 10)
        #self.subscription
        self.get_logger().info("convert_xyz node has been started.")
        self.publisher = self.create_publisher(Positiondegrees, 'degreecontrol_top', 10)
        self.create_timer(0.2, self.timer_callback)
        #self.create_timer(0.5, self.callback_robot)
        self.subscription = self.create_subscription(Inversedegrees, "deg_top", self.callback_inverse, 10)
        self.subscription = self.create_subscription(Forwarddegrees, "deg2_top", self.callback_forward, 10)

    def callback_robot(self, msg):

        #self.get_logger().info(msg.data)
        global inverse_J1
        global inverse_J2
        global inverse_J3
        global inverse_J4
        global plusdeg_J1
        global plusdeg_J2
        global plusdeg_J3
        global plusdeg_J4
        global position_J1
        global position_J2
        global position_J3
        global position_J4
        if msg.data == "b'L'":
            position_J1 = offset_J1 + position_J1 + interval_J1
            #position_J1 = interval_J1
        elif msg.data == "b'R'":
            position_J1 = offset_J1 + position_J1 - interval_J1
            #position_J1 = -interval_J1
        elif msg.data == "b'F'":
            #position_J2 = offset_J2 + position_J2 + interval_J2
            position_J2 = interval_J2
        elif msg.data == "b'B'":
            #position_J2 = offset_J2 + position_J2 - interval_J2
            position_J2 = -interval_J2
        elif msg.data == "b'U'":
            position_J3 = offset_J3 + position_J3 + interval_J3
        elif msg.data == "b'D'":
            position_J3 = offset_J3 + position_J3 - interval_J3
        elif msg.data == "b'S'":
            position_J4 = offset_J4 + position_J4 + interval_J4
        elif msg.data == "b'X'":
            position_J4 = offset_J4 + position_J4 - interval_J4
        elif msg.data == "b'E'":
            print('sum in',(inverse_J1+inverse_J2+inverse_J3+inverse_J4))
            if (inverse_J1+inverse_J2+inverse_J3+inverse_J4) == 0.0:
                position_J1 = plusdeg_J1
                position_J2 = plusdeg_J2
                position_J3 = plusdeg_J3
                position_J4 = plusdeg_J4
            else:
                position_J1 = inverse_J1
                position_J2 = inverse_J2
                position_J3 = inverse_J3
                position_J4 = inverse_J4
    
    def timer_callback(self):
        global position_J1
        global position_J2
        global position_J3
        global position_J4
        msg = Positiondegrees()
        msg.positionj_1 = position_J1
        msg.positionj_2 = position_J2
        msg.positionj_3 = position_J3
        msg.positionj_4 = position_J4
        self.publisher.publish(msg)
        self.get_logger().info('Publishing position_J1: "%f"' % msg.positionj_1) 
        self.get_logger().info('Publishing position_J2: "%f"' % msg.positionj_2) 
        self.get_logger().info('Publishing position_J3: "%f"' % msg.positionj_3) 
        self.get_logger().info('Publishing position_J4: "%f"' % msg.positionj_4) 
    
    def callback_inverse(self, msg):
        global inverse_J1
        global inverse_J2
        global inverse_J3
        global inverse_J4

        inverse_J1 = msg.inversej1
        inverse_J2 = msg.inversej2
        inverse_J3 = msg.inversej3
        inverse_J4 = msg.inversej4

    def callback_forward(self, msg):
        global plusdeg_J1
        global plusdeg_J2
        global plusdeg_J3
        global plusdeg_J4
        
        plusdeg_J1 = msg.forwardj1
        plusdeg_J2 = msg.forwardj2
        plusdeg_J3 = msg.forwardj3
        plusdeg_J4 = msg.forwardj4
        
        self.get_logger().info('sub position_J1: "%f"' % msg.forwardj1) 
        
        

def main(args=None):
    rclpy.init(args=args)
    node = convert_degrees()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
