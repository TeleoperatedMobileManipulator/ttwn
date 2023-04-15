import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32

class networkbridge(Node):

    def __init__(self):
        super().__init__('networkbridge')
        self.publisher = self.create_publisher(Int32, 'webint_top', 10)
        self.get_logger().info("networkbridge node has been started.")
        #self.publisher2 = self.create_publisher(Int32, 'commu', 10)
        self.create_timer(1.0, self.timer_callback)
        self.subscriber_ = self.create_subscription(Float32, "degerr_top", self.callback_err, 10)

    def timer_callback(self):
        msg = Int32()
        msg.data = 180
        #msg2 = Int32()
        #msg2.data = 120
        self.publisher.publish(msg)
        #self.publisher.publish(msg2)
    
    def callback_err(self, msg):
        print(msg.data)
    
def main(args=None):
    rclpy.init(args=args)
    _node = networkbridge()
    rclpy.spin(_node)
    rclpy.shutdown()

if __name__== '__main__':
    main()