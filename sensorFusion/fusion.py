import math
import numpy as np
import rospy
import matplotlib.pyplot as plt
import time
from std_msgs.msg import Float32MultiArray
import signal
import sys
import socket
import tf


# GLOBAL VAR
Q = 0.2**2 # process variance
Q = np.array([[Q, 0], [0, Q]])
R = 0.3**2 # estimate of measurement variance, change to see effect
R = np.array([[R, 0], [0, R ]])

## TODO combine socket with this fusion code

class poseEstimation:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.timestamp = 0
        self.newState = True #true when cam data is new

class Kalman:
    def __init__(self):
        self.K = np.zeros(shape= (2, 2))
        self.xhat=np.zeros(shape= (2, 1))      # a posteri estimate of []
        self.P=np.zeros(shape= (2, 2))         # a posteri error estimate
        self.xhatminus=np.zeros( shape= (2, 1) ) # a priori estimate of x
        self.Pminus=np.zeros( shape= (2, 2) )    # a priori error estimate
        self.z_minus=np.zeros( shape= (2, 1) )
        self.t_minus=0

class plot:
    def __init__(self):
        self.timeminus = 0
        self.x_minus = 0
        self.y_minus = 0


def mytopic_callback(msg):
    cam_msg = msg.data
    
    cam.x = cam_msg[0],
    cam.y = cam_msg[1]
    cam.yaw = cam_msg[2]
    cam.timestamp = time.time()
    cam.newState = True
    print "  Callback", cam_msg[0], cam_msg[1], cam_msg[2]


def plotGraph(time, x_fusion, y_fusion):
    plt.subplot(2, 1, 1)
    plt.title('x-displacement')
    if cam.newState == True:
        plt.plot(time, cam.x, 'r.', markersize=12, label = "vision")
    plt.plot([plotKalman.timeminus, time], [plotKalman.x_minus, x_fusion.xhat[0][0]], 'b-', label = "kalman") #line
    
    plt.subplot(2, 1, 2)
    plt.title('y-displacement')
    if cam.newState == True:
        plt.plot(time, cam.y, 'c.-', markersize=10, label = "vision")
    plt.plot([plotKalman.timeminus, time], [plotKalman.y_minus, y_fusion.xhat[0][0]], 'm-', label = "kalman") #line

    
    # update for prev for nxt
    plotKalman.x_minus = x_fusion.xhat[0][0]
    plotKalman.y_minus = y_fusion.xhat[0][0]
    plotKalman.timeminus = time
    cam.newState = False        #finsih reading, prepare for new one
    plt.pause(0.01)


def ROS_publishResults():
    x = x_fusion.xhat[0][0]
    y = y_fusion.xhat[0][0]
    yaw = cam.yaw
    br.sendTransform((x, y, 0), tf.transformations.quaternion_from_euler(1.571,0, yaw), rospy.Time.now(), '/base_link',"/world")
    cam.newState = False  


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


def kalmanFusion(obj, cam_displacement, imu_accel, timestamp, state):
    time_diff = timestamp - obj.t_minus
    obj.t_minus = timestamp
    A_matrix = np.array([[1, time_diff], [0,  1]])
    B_matrix = np.array([[time_diff*time_diff/2], [time_diff]])

    # =================time update (imu dependency) ====================
    obj.xhatminus = np.matmul( A_matrix, obj.xhat ) + imu_accel * B_matrix #equation 7
    obj.Pminus = obj.P + Q

    #measurenment update from camera
    if state == True:
        # ==================== measurement update (vision) =========================
        obj.z_minus[1] = (cam_displacement - obj.z_minus[0])/time_diff     #update speed
        obj.z_minus[0] = cam_displacement
        # ================= measurement update ===========================
        obj.xhat = obj.xhatminus + np.matmul(obj.K, obj.z_minus - obj.xhatminus)    #equation 18
        obj.K = np.matmul(obj.Pminus, np.linalg.inv( obj.Pminus+R ))   #equation 19
        obj.P = (1-obj.K)*obj.Pminus
    else:
        obj.xhat = obj.xhatminus #update displacemnt profile when no measurement update


if __name__=="__main__":
    
    signal.signal(signal.SIGINT, signal_handler)

    cam = poseEstimation()
    plotKalman = plot()

    x_fusion = Kalman()
    y_fusion = Kalman()

    rospy.init_node("Fusion")
    br = tf.TransformBroadcaster()
    mysub = rospy.Subscriber('cam_poseEstimation', Float32MultiArray, mytopic_callback)
    startTime = time.time()

    plt.figure("Odometry ")
    plt.axis([0, 30, 0, 5])

    while 1:

        timeFromStart = time.time() - startTime
        cam.timestamp= time.time() #NOTE!! need to remove when integration
        
        kalmanFusion(x_fusion, cam.x, 0, cam.timestamp, cam.newState)
        kalmanFusion(y_fusion, cam.y, 0, cam.timestamp, cam.newState)

        print "xhat: : ", x_fusion.xhat.transpose()
        print "zminus: ", x_fusion.z_minus.transpose()
        print "From Main {} {} {} | {}".format(cam.x, cam.y, cam.yaw, timeFromStart)
        
        # === publish results ===
        plotGraph(timeFromStart, x_fusion, y_fusion)
        # ROS_publishResults()
        rospy.sleep(0.1)

    rospy.spin()