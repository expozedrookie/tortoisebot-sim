
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import isinf,isnan,pi,exp



class ParameterTestNode(Node):

    def __init__(self):
        super().__init__('ball_follow_node') # node name
        
        self.publisher = self.create_publisher(Twist,'cmd_vel',10) #publishing to /cmd_vel
        self.get_logger().warn('[DEBUG][BALL_FOLLOW_NODE] STARTED')
        self.get_logger().info('[DEBUG][BALL_FOLLOW_NODE] Publishing to /cmd_vel')
        self.declare_parameter('rotate','right')
        timer_period = 0.1  # 50 milliseconds
        self.timer = self.create_timer(timer_period, self.cmdvel_callback)
        
        
    def cmdvel_callback(self):


        v=self.get_parameter('rotate').value
        outputCmdVel=Twist()
        if v == 'right':
            outputCmdVel.angular.z=1.0
        if v == 'left':
            outputCmdVel.angular.z=-1.0
        
        outputCmdVel.linear.x=0.0
        self.publisher.publish(outputCmdVel)
            

def main(args=None):
    import subprocess
    cmd = [
    "ros2", "topic", "pub", "--once", "/cmd_vel", "geometry_msgs/msg/Twist",
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    ]
    

    rclpy.init(args=args)
    lidarsubb=ParameterTestNode()
    try:
        rclpy.spin(lidarsubb)
    except KeyboardInterrupt:
        try:
            lidarsubb.destroy_node()
            rclpy.shutdown()
        except rclpy._rclpy_pybind11.RCLError:
            print('\n NODE ALREADY SHUTDOWN')
    finally:
        print('STOPPING ROBOT')
        subprocess.run(cmd)
        

if __name__=='__main__':
    main()
