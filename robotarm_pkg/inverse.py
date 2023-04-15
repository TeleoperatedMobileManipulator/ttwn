import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from numpy import *
from robotarm_interfaces.msg import Positiondegrees
from robotarm_interfaces.msg import Inversedegrees
from robotarm_interfaces.msg import Sensordeg
import firebase_admin
from firebase_admin import credentials, firestore
import time
import threading


x_position = None
y_position = None
z_position = None
# Config
# Load the service account key file
cred = credentials.Certificate('/home/tawanpc/ros2ws/src/robotarm_pkg/robotarm_pkg/serviceAccoutKey.json')

#offset
theta_1deg = 0.0
theta_2deg = 0.0
theta_3deg = 0.0
theta_4deg = 0.0

senj1 = 0.0
senj2 = 0.0
senj1 = 0.0


class inverse(Node):

    def __init__(self):
        super().__init__('inverse')
        self.get_logger().info("inverse node has been started.")
        self.publisher = self.create_publisher(Inversedegrees, 'deg_top', 10)
        self.subscriber_ = self.create_subscription(Sensordeg, "sensordegre_top", self.callback_readdeg, 10)
        self.create_timer(1.0, self.timer_callback)       

    def callback_readdeg(self,msg):
        global senj2
        global senj3
        global senj1
        senj1 = msg.sendegre1
        senj2 = msg.sendegre2
        senj3 = msg.sendegre3
    def timer_callback(self):
        # Length of links in mm
        d1 = 180
        a1 = 10
        a2 = 225
        a3 = 230
        a4 = 85

        global x_position
        global y_position
        global z_position

        msg = Inversedegrees()

        # Desired Position of End effector in mm
        print('x || y|| z', x_position, y_position, z_position)
        print(' state x || y|| z', x_position and y_position and z_position )
        print(type(x_position and y_position and z_position))
        if (x_position or y_position or z_position) == None:
            print('----------------------NOne')
            msg.inversej1 = 0.0
            msg.inversej2 = 0.0
            msg.inversej3 = 0.0
            msg.inversej4 = 0.0
        elif (x_position or y_position or z_position) != None:
            print('--------+++++++++--------------')
            px = x_position
            py = y_position
            pz = z_position

            psi = 270
            psi = deg2rad(psi)

            thp2=90
            thp2= deg2rad(thp2)

            s=180
            s= deg2rad(s)

            # Equations for Inverse kinematics
            theta_1= arctan2(py,px)  #eq1
            r1=sqrt(px**2+py**2)   #eq2

            r2=pz-d1-(a4*sin(psi)) #eq3
            phi_2=arctan2(r2,r1) #eq4
            r3=sqrt(r1**2+r2**2) #eq5
            phi_1=arccos((a3**2-a2**2-r3**2)/(-2*a2*r3)) #eq6
            theta_2=phi_2+phi_1 #eq7

            phi_3=arccos((r3**2-a2**2-a3**2)/(-2*a2*a3))
            theta_3=s-phi_3

            theta_4=psi+theta_2-theta_3

            
            global theta_1deg
            global theta_2deg
            global theta_3deg
            global theta_4deg
            print('bf1: ',theta_1deg)
            print('bf2: ',theta_2deg)
            print('bf3: ',theta_3deg)
            print('bf4: ',theta_4deg)
            theta_1deg = rad2deg(theta_1)
            theta_2deg = rad2deg(theta_2)
            theta_3deg = rad2deg(theta_3)
            theta_3deg = - theta_3deg

            print('af1: ',theta_1deg)
            print('af2: ',theta_2deg)
            print('af3: ',theta_3deg)
            print('af4: ',theta_4deg)
            
        
            '''
            if theta_2 >= 89 or theta_2 < -91 or theta_3 >= 2 or theta_3 < -130 :
                msg.inversej1 = 0.0
                msg.inversej2 = 0.0
                msg.inversej3 = 0.0
                msg.inversej4 = 0.0
            '''

            if (isnan(theta_1deg) or isnan(theta_2deg) or isnan(theta_3deg)) == True:
                msg.inversej1 = 0.0
                msg.inversej2 = 0.0
                msg.inversej3 = 0.0
                msg.inversej4 = 0.0
            else :
                global senj2
                global senj3
                global senj1
                msg.inversej1 = theta_1deg - senj1
                msg.inversej2 = theta_2deg - senj2   
                msg.inversej3 = theta_3deg - senj3
                msg.inversej4 = 0.0
                global inv1
                global inv2
                global inv3
                '''
                def back():
                    msg.inversej1 = inv1
                    msg.inversej2 = inv2           
                    msg.inversej3 = inv3   
                    msg.inversej4 = 0.0
                    print ("malaew",inv1)
                    self.publisher.publish(msg)
                    self.get_logger().info('Publishing position_J1: "%f"' % msg.inversej1) 
                    self.get_logger().info('Publishing position_J2: "%f"' % msg.inversej2) 
                    self.get_logger().info('Publishing position_J3: "%f"' % msg.inversej3) 
                    self.get_logger().info('Publishing position_J4: "%f"' % msg.inversej4)
               
                inv1 = -theta_1rad
                inv2 = -theta_2rad
                inv3 = -theta_3rad
                timer = threading.Timer(5.0, back)
                timer.start()
                '''
                
                
            


        self.publisher.publish(msg)
        self.get_logger().info('Publishing position_J1: "%f"' % msg.inversej1) 
        self.get_logger().info('Publishing position_J2: "%f"' % msg.inversej2) 
        self.get_logger().info('Publishing position_J3: "%f"' % msg.inversej3) 
        self.get_logger().info('Publishing position_J4: "%f"' % msg.inversej4) 
        x_position = None
        y_position = None
        z_position = None


def handle_Position(doc_snapshot, changes, read_time):
    data = firestore_ref.collection('position').document('coordinate').get()
    coordinate = data.to_dict()
    global x_position
    global y_position
    global z_position
    print('coordinate: ',coordinate)
    x_position_str = coordinate['x_position']
    y_position_str = coordinate['y_position']
    z_position_str = coordinate['z_position']
    
    try:
        x_position = float(x_position_str)
        y_position = float(y_position_str)
        z_position = float(z_position_str)
    except ValueError:
        x_position = None
        y_position = None
        z_position = None
    

  

def main(args=None):
    #Initialize the app with the service account
    firebase_admin.initialize_app(cred)

    #Get a referance to the firestore
    global firestore_ref
    firestore_ref = firebase_admin.firestore.client(app=None) 
    print(firestore_ref)

    coordinate_set = {
            'x_position': 'None',
            'y_position': 'None',
            'z_position': 'None'
            }
    firestore_ref.collection('position').document('coordinate').set(coordinate_set)

    position_fire = firestore_ref.collection('position').document('coordinate')
    docs = position_fire.on_snapshot(lambda doc_snapshot, changes, read_time: handle_Position(doc_snapshot, changes, read_time))
    

    rclpy.init(args=args)
    _node = inverse()
    rclpy.spin(_node)
    rclpy.shutdown()

if __name__== '__main__':
    main()