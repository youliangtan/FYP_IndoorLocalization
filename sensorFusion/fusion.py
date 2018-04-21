import math
import numpy as np
import rospy
import matplotlib.pyplot as plt
import time
from std_msgs.msg import Float32MultiArray
import signal
import sys


## TODO combine socket with this fusion code

#Global Var
class poseEstimation:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.timestamp = 0

def mytopic_callback(msg):
    cam_msg = msg.data
    
    cam.x = cam_msg[0],
    cam.y = cam_msg[1]
    cam.yaw = cam_msg[2]
    cam.timestamp = time.time()
    print "  Callback", cam_msg[0], cam_msg[1], cam_msg[2]


def plotGraph(time):
    plt.subplot(2, 1, 1)
    plt.title('x-displacement')
    plt.plot(time, cam.x, 'go-', label = "truth")
    plt.subplot(2, 1, 2)
    plt.title('y-displacement')
    plt.plot(time, cam.y, 'ro-', label = "truth")
    plt.pause(0.05)

def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)



if __name__=="__main__":
    
    signal.signal(signal.SIGINT, signal_handler)

    cam = poseEstimation()
    rospy.init_node("Fusion")
    mysub = rospy.Subscriber('cam_poseEstimation', Float32MultiArray, mytopic_callback)
    startTime = time.time()

    plt.figure("Odometry ")
    plt.axis([0, 30, 0, 5])

    while 1:
        # rospy.sleep(0.1)

        timeFromStart = time.time() - startTime
        plotGraph(timeFromStart)

        print "From Main {} {} {} | {}".format(cam.x, cam.y, cam.yaw, timeFromStart)

        



    rospy.spin()

