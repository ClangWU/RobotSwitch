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
Pose = PoseStamped()

def callback_function(msg):  
    global msg_obs       
    msg_obs = msg.data
    # print("msg_obs:", msg_obs)

def message_thread():
    obs_sub = rospy.Subscriber('/observation', Float32MultiArray, callback_function)
    rospy.spin()

def zmq_action(socket):


    # 定义一个浮点数数组
    # float_array = [4.4, 5.5]
    # 将浮点数数组编码为JSON字符串
    # message = json.dumps(float_array)
    message = "send action"
    socket.send_string(message)

    # 等待服务器响应
    reply = socket.recv_string()
    float_array = json.loads(reply)
    Pose.pose.position.x = float_array[0]
    Pose.pose.position.y = float_array[1]
    Pose.pose.position.z = float_array[2]
    Pose.pose.orientation.w = float_array[3]
    Pose.pose.orientation.x = float_array[4]
    Pose.pose.orientation.y = float_array[5]
    Pose.pose.orientation.z = float_array[6]
    action_pub.publish(Pose)
    # print(f"Received reply: {reply}")

def zmq_obs(socket):
    # 等待客户端消息
    message = socket.recv_string()
    # obs = json.loads(message)  # 将接收到的字符串转换回浮点数数组
    print(message)
    obs_array = np.array(msg_obs)
    message = json.dumps(obs_array.tolist())
    socket.send_string(message)

if __name__ == "__main__":
    rospy.init_node('real_env_node', anonymous=True)
    action_pub = rospy.Publisher('/cartesian_impedance_controller/desired_pose', PoseStamped, queue_size=10)
    msg_thread = threading.Thread(target=message_thread)
    msg_thread.start()
    context = zmq.Context()
    socket_obs = context.socket(zmq.REP)
    socket_obs.bind("tcp://*:5555")  # 绑定端口5555
    
    context = zmq.Context()
    socket_act = context.socket(zmq.REQ)
    socket_act.connect("tcp://192.168.1.102:5555")  # 连接到服务器

    while True:
        zmq_action(socket_act)
        zmq_obs(socket_obs)

# import zmq
# import time
# import rospy
# import struct
# import threading

# def zmq_subscriber_thread():
#     context = zmq.Context()
#     socket = context.socket(zmq.PUSH)
#     socket.bind("tcp://*:5556")
#     while True:
#         socket.send(b"Hello from PUSH")

# def zmq_publisher_thread():
#     context = zmq.Context()
#     socket = context.socket(zmq.PULL)
#     socket.connect("tcp://192.168.1.102:5556")
#     while True:
#         message = socket.recv()
#         print(f"Received request: {message}")


# if __name__ == '__main__':
    
#     zmq_pub_thread = threading.Thread(target=zmq_publisher_thread)
#     zmq_pub_thread.start()

#     zmq_sub_thread = threading.Thread(target=zmq_subscriber_thread)
#     zmq_sub_thread.start()
