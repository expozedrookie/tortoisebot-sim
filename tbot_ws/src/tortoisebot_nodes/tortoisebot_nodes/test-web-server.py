import os
import rclpy
import threading

from flask import Flask

from std_msgs.msg import UInt32

def ros_callback(msg):
    print(msg)

threading.Thread(target=lambda: rclpy.init_node('example_node', disable_signals=True)).start()
rclpy.Subscriber('/listener', UInt32, ros_callback)
pub = rclpy.Publisher('/talker', UInt32, queue_size=10)

app = Flask(__name__)

@app.route('/')
def hello_world():
    msg = UInt32()
    msg.data = 1
    pub.publish(msg)
    
    return 'Hello, World!'

if __name__ == '__main__':
    app.run(host=os.environ['ROS_IP'], port=3000)



