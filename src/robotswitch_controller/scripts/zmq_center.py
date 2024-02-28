import zmq
import time
import rospy
import json
import numpy as np
import threading
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

msg_obs_lock = threading.Lock()
msg_obs = []
def callback_function(msg):  
    global msg_obs       
    msg_obs = msg.data
    # print("msg_obs:", msg_obs)

def message_thread():
    obs_sub = rospy.Subscriber('/observation', Float32MultiArray, callback_function)
    rospy.spin()

def zmq_client():
    context = zmq.Context()
    print("Connecting to server...")
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://192.168.1.102:5555")  # 连接到服务器

    # 定义一个浮点数数组
    while True:
        time.sleep(0.00075)
        # 等待服务器响应
        reply = socket.recv_string()
        print(f"Received reply: {reply}")
        # 收到消息后，发送消息
        obs_array = np.array(msg_obs)
        message = json.dumps(obs_array.tolist())
        socket.send_string(message)

if __name__ == '__main__':
    rospy.init_node('real_env_node', anonymous=True)
    action_pub = rospy.Publisher('/cartesian_impedance_controller/desired_pose', PoseStamped, queue_size=10)

    msg_thread = threading.Thread(target=message_thread)
    msg_thread.start()

    zmq_thread = threading.Thread(target=zmq_client)
    zmq_thread.start()