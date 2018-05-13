#       ___     MODEL: IMU L3GD20
#      /00/|                
#     /00/ |                  y-axis 
#    |  |  |__________      /
#    |  |  /         /|    /===> x-axis (reference north dir)
#    |__| /________ //
#    |__|__________|/

# TODO: fix change orientation acceleration (accumulateed acceleration prob for IMU frame), 
# add feature of IMU x-y axis to world xy calib, 
# timediff of IMU accel references
# All code run on roslaunch
# create never end imu reading

#reference for me youliang: http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/


#fusion1: 3 threads which one with 2 ROS subscriber 1 one publisher
#fusion2: 2 callbacks and 1 main
#fusion3: 3 callbacks and 1 main 


import math
import numpy as np
import rospy
import matplotlib.pyplot as plt
import time
from std_msgs.msg import Float32MultiArray
import signal
import sys
import tf

# GLOBAL VAR
Q = 0.2**2 # process variance
Q = np.array([[Q, 0], [0, Q]])
R = 0.3**2 # estimate of measurement variance, change to see effect
R = np.array([[R, 0], [0, R ]])
terminalVel_reduction_factor = 0.8



class poseEstimation:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.timestamp = 0

class Kalman:
    def __init__(self):
        self.K = np.zeros(shape= (2, 2))
        self.xhat=np.zeros(shape= (2, 1))      # a posteri estimate of []
        self.P=np.zeros(shape= (2, 2))         # a posteri error estimate
        self.xhatminus=np.zeros( shape= (2, 1) ) # a priori estimate of x
        self.Pminus=np.zeros( shape= (2, 2) )    # a priori error estimate
        self.z_minus=np.zeros( shape= (2, 1) )
        self.t_minus=0
        self.camDataState = False


class plot:
    def __init__(self):
        self.timeminus = 0
        self.x_minus = 0
        self.y_minus = 0
        self.newCamData = False

class IMUAccel:
    def __init__(self):
        self.x_accel = 0
        self.y_accel = 0
        self.yaw = 0


def plotGraph(time, x_fusion, y_fusion):
    plt.subplot(2, 1, 1)
    plt.title('x-displacement')
    if plotKalman.newCamData == True:
        plt.plot(time, cam.x, 'r.', markersize=12, label = "vision")
    plt.plot([plotKalman.timeminus, time], [plotKalman.x_minus, x_fusion.xhat[0][0]], 'b-', label = "kalman") #line
    
    plt.subplot(2, 1, 2)
    plt.title('y-displacement')
    if plotKalman.newCamData == True:
        plt.plot(time, cam.y, 'c.-', markersize=10, label = "vision")
    plt.plot([plotKalman.timeminus, time], [plotKalman.y_minus, y_fusion.xhat[0][0]], 'm-', label = "kalman") #line

    
    # update for prev for nxt
    plotKalman.x_minus = x_fusion.xhat[0][0]
    plotKalman.y_minus = y_fusion.xhat[0][0]
    plotKalman.timeminus = time
    plotKalman.newCamData = False        #finsih reading, prepare for new one
    plt.pause(0.01)


def ROS_publishResults():
    x = x_fusion.xhat[0][0]
    y = y_fusion.xhat[0][0]
    yaw = imu.yaw
    br.sendTransform((x, y, 0), tf.transformations.quaternion_from_euler(1.571,0, yaw), rospy.Time.now(), '/base_link',"/world")


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


def kalmanFusion(obj, cam_displacement, imu_accel, timestamp):

    state = obj.camDataState

    time_diff = timestamp - obj.t_minus
    obj.t_minus = timestamp
    A_matrix = np.array([[1, time_diff], [0,  1]])
    B_matrix = np.array([[time_diff*time_diff/2], [time_diff]])

    # =================time update (imu dependency) ====================
    obj.xhat[1] = obj.xhat[1] * terminalVel_reduction_factor  #reduce vel every iteration
    obj.xhatminus = np.matmul( A_matrix, obj.xhat ) + imu_accel * B_matrix #equation 7
    obj.Pminus = np.matmul( np.matmul(A_matrix, obj.P) , np.linalg.inv( A_matrix)) + Q    # P = A*P*A^T + Q

    #measurenment update from camera
    if state == True:
        # ==================== measurement update (vision) =========================
        obj.z_minus[1] = (cam_displacement - obj.z_minus[0])/time_diff * 0.9    #update speed*with a reducing factor from camera
        obj.z_minus[0] = cam_displacement
        # ================= measurement update ===========================
        obj.xhat = obj.xhatminus + np.matmul(obj.K, obj.z_minus - obj.xhatminus)    #equation 18
        obj.K = np.matmul(obj.Pminus, np.linalg.inv( obj.Pminus+R ))   #equation 19
        obj.P = (1-obj.K)*obj.Pminus
        #change state to indicate read alr
        obj.camDataState = False
    else:
        obj.xhat = obj.xhatminus #update displacemnt profile when no measurement update




def imu_callback(msg):
    if (time.time() - startTime > 2): #wait all initialize without callback
        imu_msg = msg.data

        # terbalik according to my room transformation
        imu.x_accel = imu_msg[1]
        imu.y_accel = -imu_msg[0]
        imu.yaw = -imu_msg[2] + math.pi -0.1 #adjust here accorinding to environment
        imu.timeDiff = imu_msg[3]

        print "  (2) Callback from IMU!", imu_msg[0], imu_msg[1], imu_msg[2]



def cam_callback(msg):
    if (time.time() - startTime > 1): #wait all initialize without callback
        cam_msg = msg.data
        
        cam.x = cam_msg[0]
        cam.y = cam_msg[1]
        cam.yaw = cam_msg[2]
        cam.timestamp = time.time()

        #reading state
        x_fusion.camDataState = True
        y_fusion.camDataState = True
        plotKalman.newCamData = True

        print "  (1) Callback from cam!", cam_msg[0], cam_msg[1], cam_msg[2]


if __name__=="__main__":

    signal.signal(signal.SIGINT, signal_handler)

    cam = poseEstimation()
    plotKalman = plot()

    x_fusion = Kalman()
    y_fusion = Kalman()
    imu = IMUAccel()

    rospy.init_node("Fusion")
    br = tf.TransformBroadcaster()

    #create call back for both subcriber
    mysub = rospy.Subscriber('cam_poseEstimation', Float32MultiArray, cam_callback)
    mysub2 = rospy.Subscriber('imu_poseEstimation', Float32MultiArray, imu_callback)
    startTime = time.time()

    plt.figure("Odometry ")
    plt.axis([0, 30, 0, 5])

    while 1:

        timeFromStart = time.time() - startTime
        cam.timestamp= time.time() #NOTE!! need to remove when integration        

        kalmanFusion(x_fusion, cam.x, imu.x_accel, cam.timestamp)
        kalmanFusion(y_fusion, cam.y, imu.y_accel, cam.timestamp)

        # print "xhat: : ", x_fusion.xhat.transpose()
        # print "zminus: ", x_fusion.z_minus.transpose()
        # print "From Main {} {} {} | {}".format(cam.x, cam.y, cam.yaw, timeFromStart)
        
        # === publish results ===
        # plotGraph(timeFromStart, x_fusion, y_fusion)
        rospy.sleep(0.1)
        ROS_publishResults()

        # pause play
        # if ch & 0xFF == ord('p'):
        #     print "\n\n Pause main thread and wait!!!!!!!\n\n"
        #     time.sleep(1)
        #     while (1):
        #         if ch & 0xFF == ord('p'):
        #             print "\n\n Run main thread and wait!!!!!!!\n\n"                    
        #             break
        #         else:
        #             pass

    rospy.spin()