#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from robotarm_interfaces.msg import Positiondegrees
from robotarm_interfaces.msg import Sensordeg
import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib
import time
import firebase_admin
from firebase_admin import credentials, firestore

cred = credentials.Certificate('/home/tawanpc/ros2ws/src/robotarm_pkg/robotarm_pkg/serviceAccoutKey.json')

direction1 = 12
step1 = 6
EN_pin1 = 5

direction2 = 16
step2 = 19
EN_pin2 = 13

direction3 = 26
step3 = 21
EN_pin3 = 20

degree1 = 0.0
degree2 = 0.0
degree3 = 0.0
sumj1 = 0.0
sumj2 = 0.0
sumj3 = 0.0

motor1 = None
motor2 = None
motor3 = None

active = None

class motor(Node):

    def __init__(self):
        global motor1
        global motor2
        global motor3

        motor1 = RpiMotorLib.A4988Nema(direction1, step1, (-1, -1, -1), "DRV8825")
        GPIO.setup(EN_pin1, GPIO.OUT)
        GPIO.output(EN_pin1, GPIO.LOW)
        print(motor1)

        motor2 = RpiMotorLib.A4988Nema(direction2, step2, (-1, -1, -1), "DRV8825")
        GPIO.setup(EN_pin2, GPIO.OUT)
        GPIO.output(EN_pin2, GPIO.LOW)
        print(motor2)

        motor3 = RpiMotorLib.A4988Nema(direction3, step3, (21, 21, 21), "DRV8825")
        GPIO.setup(EN_pin3, GPIO.OUT)
        GPIO.output(EN_pin3, GPIO.LOW)
        print(motor3)

        super().__init__("motor")
        self.get_logger().info("motor node has been started.")
        self.subscriber_ = self.create_subscription(Sensordeg, "sensordegre_top", self.callback_sensor, 10)
        self.subscriber_ = self.create_subscription(Positiondegrees, "degreecontrol_top", self.callback_robot, 10)
        self.publisher = self.create_publisher(Float32, 'degj1_top', 10)
        self.create_timer(1.0, self.timer_j1)

    def timer_j1(self):
        global sumj1
        msg = Float32()
        msg.data = sumj1*1.8/6.9
        self.get_logger().info(("deg1 " + str(msg.data)))
        self.publisher.publish(msg)

        # self.create_timer(1.0, self.timer_callback)
    def callback_sensor(self,msg):
        global sumj2
        global sumj3
        
        sumj2 = msg.sendegre2
        sumj3 = msg.sendegre3
        

    def callback_robot(self, msg):
        self.get_logger().info(("J1 Moving to " + str(msg.positionj_1)))
        self.get_logger().info(("J2 Moving to " + str(msg.positionj_2)))
        self.get_logger().info(("J3 Moving to " + str(msg.positionj_3)))

        global degree1
        global degree2
        global degree3
        global sumj1
        global sumj2
        global sumj3
        
        degree1 = msg.positionj_1
        fldegree1 = float(degree1)
        mstep1 = fldegree1 * 6.9 / 1.8
        instep1 = int(mstep1)

        degre2 = msg.positionj_2
        degree2 = int(degre2)
        fldegree2 = float(degree2)
        mstep2 = fldegree2 * 51 / 1.8
        instep2 = int(mstep2)

        degre3 = msg.positionj_3
        degree3 = int(degre3)
        fldegree3 = float(degree3)
        mstep3 = fldegree3 * 27 / 1.8
        instep3 = int(mstep3)

        if instep1 >= 0:
            if instep1 == 0:
                GPIO.output(EN_pin1, GPIO.HIGH)
                GPIO.output(EN_pin1, GPIO.LOW)
            motor1.motor_go(True,  # True=Clockwise, False=Counter-Clockwise
                            "Full",
                            instep1,  # number of steps
                            .015,  # step delay [sec]
                            False,  # True = print verbose output
                            .05)  # initial delay [sec]
            sumj1 += instep1

        elif instep1 < 0:
            instep1 = abs(instep1)
            motor1.motor_go(False,
                            "Full",
                            instep1,
                            .015,
                            False,
                            .05)
            sumj1 -= instep1

        if instep2 >= 0:
            if instep2 == 0:
                GPIO.output(EN_pin2, GPIO.HIGH)
                GPIO.output(EN_pin2, GPIO.LOW)
            motor2.motor_go(True,
                            "Full",
                            instep2,
                            .008,
                            False,
                            .05)
            
        elif instep2 < 0:
            instep2 = abs(instep2)
            motor2.motor_go(False,
                            "Full",
                            instep2,
                            .008,
                            False,
                            .05)

        if instep3 >= 0:
            if instep3 == 0:
                GPIO.output(EN_pin3, GPIO.HIGH)
                GPIO.output(EN_pin3, GPIO.LOW)
            motor3.motor_go(True,
                            "Full",
                            instep3,
                            .01,
                            False,
                            .05)

        elif instep3 < 0:
            instep3 = abs(instep3)
            motor3.motor_go(False,
                            "Full",
                            instep3,
                            .01,
                            False,
                            .05)
        #self.home(sumj1, sumj2, sumj3)
        global active
        if active != None:
            if active['active'] == 'active' :
                
                backj1 = int(-sumj1)
                backj2 = int(-sumj2*51/1.8)
                backj3 = int(-sumj3*27/1.8)
                if backj1 >= 0:
                    motor1.motor_go(True,  # True=Clockwise, False=Counter-Clockwise
                                    "Full",
                                    backj1,  # number of steps
                                    .015,  # step delay [sec]
                                    False,  # True = print verbose output
                                    .05)  # initial delay [sec]
                elif backj1 < 0:
                    instep1 = abs(backj1)
                    motor1.motor_go(False,
                                    "Full",
                                    instep1,
                                    .015,
                                    False,
                                    .05)
                if backj3 >= 0:
                    motor3.motor_go(True,
                                    "Full",
                                    backj3,
                                    .01,
                                    False,
                                    .05)
                elif backj3 < 0:
                    instep3 = abs(backj3)
                    motor3.motor_go(False,
                                    "Full",
                                    instep3,
                                    .01,
                                    False,
                                    .05)
                if backj2 >= 0:
                    motor2.motor_go(True,
                                    "Full",
                                    backj2,
                                    .008,
                                    False,
                                    .05)
                elif backj2 < 0:
                    instep2 = abs(backj2)
                    motor2.motor_go(False,
                                    "Full",
                                    instep2,
                                    .008,
                                    False,
                                    .05)
                sumj1 = 0.0
                
                print(("J1 back to " + str(backj1)))
                print(("J2 back to " + str(backj2)))
                print(("J3 back to " + str(backj3)))
                state = {
                    'active': 'notactive',
                    
                    }
                firestore_ref.collection('control').document('home').set(state)
                print('home complete')

def handle_home(doc_snapshot, changes, read_time):
    global active
    data = firestore_ref.collection('control').document('home').get()
    active = data.to_dict()
    print(active)
    
        


def main(args=None):
    # Initialize the app with the service account
    firebase_admin.initialize_app(cred)

    # Get a referance to the firestore
    global firestore_ref
    firestore_ref = firebase_admin.firestore.client(app=None)
    print(firestore_ref)

    position_fire = firestore_ref.collection('control').document('home')
    docs = position_fire.on_snapshot(
        lambda doc_snapshot, changes, read_time: handle_home(doc_snapshot, changes, read_time))

    rclpy.init(args=args)
    node = motor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
