import rclpy
from rclpy.lifecycle import LifecycleNode,State,TransitionCallbackReturn
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry


class robotBringUp(LifecycleNode):
    def __init__(self):
        super().__init__('tbot_super_lifecycle')
        # parameters
        self.declare_parameter('p1',1.0)
        self.subscriber=None
        self.msgData=None
        self.firstRun=True
        self.prev_x='0'
        self.prev_y='0'

    def on_configure(self,state:State):
        self.get_logger().info('configuring ')
        self. maxp=self.get_parameter('p1').value
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self,state:State):
        self.get_logger().info('activating ')
        self.subscriber=self.create_subscription(Odometry,'odom',self.listener_callback,10)
        self.publisher_callback()
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state:State):
        self.get_logger().info('deactivating ')
        self.subscriber=None
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self,state:State):
        self.get_logger().info('cleaning up  ')
        return TransitionCallbackReturn.SUCCESS
    
    
    def listener_callback(self,msg):
        self.msgData=msg
        odomType=Odometry()
        if self.firstRun:
            self.prev_x='0'
            self.prev_y='0'
        self.get_logger().info('got odom data %s'%msg)

    
    def publisher_callback(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = robotBringUp()

    # Typical lifecycle transitions
    node.on_configure(None)
    node.on_activate(None)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_deactivate(None)
        node.on_cleanup(None)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()