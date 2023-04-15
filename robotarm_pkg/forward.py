import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from numpy import *
from robotarm_interfaces.msg import Positiondegrees
from robotarm_interfaces.msg import Inversedegrees
from robotarm_interfaces.msg import Forwarddegrees
import firebase_admin
from firebase_admin import credentials, firestore
from robotarm_interfaces.msg import Sensordeg


x_position = None
y_position = None
z_position = None
x = 0.0
y = 0.0
z = 0.0
# Config
# Load the service account key file
cred = credentials.Certificate('/home/tawanpc/ros2ws/src/robotarm_pkg/robotarm_pkg/serviceAccoutKey.json')

class forward(Node):

    def __init__(self):
        super().__init__('forward')
        self.get_logger().info("forward node has been started.")
        self.publisher_ = self.create_publisher(Forwarddegrees, 'deg2_top', 10)
        self.publisher = self.create_publisher(Inversedegrees, 'realxyz_top', 10)
        self.subscriber_ = self.create_subscription(Sensordeg, "sensordegre_top", self.callback_xyz, 10)
        self.create_timer(1.0, self.timer_callback)
        self.create_timer(1.0, self.timer_realxyz)

    def callback_xyz(self,msg):
        global senj2
        global senj3
        global senj1
        global y
        global z
        global x

        senj1 = msg.sendegre1
        senj2 = msg.sendegre2
        senj3 = msg.sendegre3
        # Length of links in mm
        d1 = 180
        d2 = 0
        d3 = 0
        d4 = 0
        a1 = 10
        a2 = 225
        a3 = 230
        a4 = 85

        # Angles
        theta_1 = 0  # theta 1 angle in degrees
        theta_2 = 0  # theta 2 angle in degrees
        theta_3 = 0  # theta 3 angle in degrees
        theta_4 = -90  # theta 4 angle in degrees

        #Input
        theta_1 = senj1
        theta_2 = senj2
        theta_3 = senj3
        theta_4 = -90

        theta_1 = (theta_1 / 180) * pi  # theta 1 in radians
        theta_2 = (theta_2 / 180) * pi  # theta 2 in radians
        theta_3 = (theta_3 / 180) * pi  # theta 3 in radians
        theta_4 = (theta_4 / 180) * pi  # theta 4 in radians

        # DH Parameter Table for 4 DOF Planar
        PT = [[theta_1, (90 / 180) * pi, a1, d1],
              [theta_2, 0, a2, d2],
              [theta_3, 0, a3, d3],
              [theta_4, 0, a4, d4]]

        # Homogeneous Transformation Matrices
        i = 0
        H0_1 = [
            [cos(PT[i][0]), -sin(PT[i][0]) * cos(PT[i][1]), sin(PT[i][0]) * sin(PT[i][1]), PT[i][2] * cos(PT[i][0])],
            [sin(PT[i][0]), cos(PT[i][0]) * cos(PT[i][1]), -cos(PT[i][0]) * sin(PT[i][1]), PT[i][2] * sin(PT[i][0])],
            [0, sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
            [0, 0, 0, 1]]

        i = 1
        H1_2 = [
            [cos(PT[i][0]), -sin(PT[i][0]) * cos(PT[i][1]), sin(PT[i][0]) * sin(PT[i][1]), PT[i][2] * cos(PT[i][0])],
            [sin(PT[i][0]), cos(PT[i][0]) * cos(PT[i][1]), -cos(PT[i][0]) * sin(PT[i][1]), PT[i][2] * sin(PT[i][0])],
            [0, sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
            [0, 0, 0, 1]]

        i = 2
        H2_3 = [
            [cos(PT[i][0]), -sin(PT[i][0]) * cos(PT[i][1]), sin(PT[i][0]) * sin(PT[i][1]), PT[i][2] * cos(PT[i][0])],
            [sin(PT[i][0]), cos(PT[i][0]) * cos(PT[i][1]), -cos(PT[i][0]) * sin(PT[i][1]), PT[i][2] * sin(PT[i][0])],
            [0, sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
            [0, 0, 0, 1]]

        i = 3
        H3_4 = [
            [cos(PT[i][0]), -sin(PT[i][0]) * cos(PT[i][1]), sin(PT[i][0]) * sin(PT[i][1]), PT[i][2] * cos(PT[i][0])],
            [sin(PT[i][0]), cos(PT[i][0]) * cos(PT[i][1]), -cos(PT[i][0]) * sin(PT[i][1]), PT[i][2] * sin(PT[i][0])],
            [0, sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
            [0, 0, 0, 1]]

        H0_2 = dot(H0_1, H1_2)
        H0_3 = dot(H0_2, H2_3)
        H0_4 = dot(H0_3, H3_4)

        print('x =', H0_4[0][3])
        print('y =', H0_4[1][3])
        print('z =', H0_4[2][3])
        x = (H0_4[0][3])
        y = (H0_4[1][3])
        z = (H0_4[2][3])

    def timer_realxyz(self):
        global x
        global y
        global z
        msg1 = Inversedegrees()
        msg1.inversej1 = x
        msg1.inversej2 = y
        msg1.inversej3 = z 
        self.publisher.publish(msg1)

    def timer_callback(self):
        global plusdegree1
        global plusdegree2
        global plusdegree3
        global senj2
        global senj3
        global senj1
        
        msg = Forwarddegrees()
        if (plusdegree1 or plusdegree2 or plusdegree3) == None:
            print('No Value Forward')
            msg.forwardj1 = 0.0
            msg.forwardj2 = 0.0
            msg.forwardj3 = 0.0
            msg.forwardj4 = 0.0
        elif (plusdegree1 or plusdegree2 or plusdegree3) != None:
            print('Doing Forward')
            msg.forwardj1 = plusdegree1
            msg.forwardj2 = plusdegree2
            msg.forwardj3 = plusdegree3
            msg.forwardj4 = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing position_J1: "%f"' % msg.forwardj1) 
        self.get_logger().info('Publishing position_J2: "%f"' % msg.forwardj2) 
        self.get_logger().info('Publishing position_J3: "%f"' % msg.forwardj3) 
        self.get_logger().info('Publishing position_J4: "%f"' % msg.forwardj4)
        plusdegree1 = 0.0
        plusdegree2 = 0.0
        plusdegree3 = 0.0
        degree_set = {
        'plusdeg1': plusdegree1,
        'plusdeg2': plusdegree2,
        'plusdeg3': plusdegree3
        }
        firestore_ref.collection('position').document('degrees').set(degree_set)
        

def handle_Position(doc_snapshot, changes, read_time):
    data = firestore_ref.collection('position').document('degrees').get()
    plusdegrees = data.to_dict()
    global plusdegree1
    global plusdegree2
    global plusdegree3
    print('plusdegrees: ', plusdegrees)
    plusdegree1_str = plusdegrees['plusdeg1']
    plusdegree2_str = plusdegrees['plusdeg2']
    plusdegree3_str = plusdegrees['plusdeg3']

    try:
        plusdegree1 = float(plusdegree1_str)
        plusdegree2 = float(plusdegree2_str)
        plusdegree3 = float(plusdegree3_str)
    except ValueError:
        plusdegree1 = 0.0
        plusdegree2 = 0.0
        plusdegree3 = 0.0


def main(args=None):
    # Initialize the app with the service account
    firebase_admin.initialize_app(cred)

    # Get a referance to the firestore
    global firestore_ref
    firestore_ref = firebase_admin.firestore.client(app=None)
    print(firestore_ref)

    degree_set = {
        'plusdeg1': 0.0,
        'plusdeg2': 0.0,
        'plusdeg3': 0.0
    }
    firestore_ref.collection('position').document('degrees').set(degree_set)

    position_fire = firestore_ref.collection('position').document('degrees')
    docs = position_fire.on_snapshot(
        lambda doc_snapshot, changes, read_time: handle_Position(doc_snapshot, changes, read_time))

    rclpy.init(args=args)
    _node = forward()
    rclpy.spin(_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()