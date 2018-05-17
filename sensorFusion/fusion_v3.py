#       ___     MODEL: IMU L3GD20
#      /00/|                
#     /00/ |                  y-axis 
#    |  |  |__________      /
#    |  |  /         /|    /===> x-axis (reference north dir)
#    |__| /________ //
#    |__|__________|/       #anticlockwise , yaw +ve

# TODO: fix change orientation acceleration (accumulateed acceleration prob for IMU frame), 
# add feature of IMU x-y axis to world xy calib, 
# timediff of IMU accel references
# All code run on roslaunch


#reference for me youliang: http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/


#fusion1: 3 threads which one is socket server
#fusion2: 2 callbacks and 1 main 
#fusion3: 3 callbacks ( + encoder ) and 1 main 


#TODO: syncronize time


import math
import numpy as np
import rospy
import matplotlib.pyplot as plt
import time
from std_msgs.msg import Float32MultiArray
import signal
import sys
import tf
import yaml

# GLOBAL VAR
_Q = 0.2**2 # process variance
Q = np.array([[_Q, 0], [0, _Q]])
_R = 0.3**2 # estimate of measurement variance, change to see effect
R = np.array([[_R, 0], [0, _R ]])
terminalVel_reduction_factor = 0.8
cam2platform_Rot = 0
yaml_path = "../markers_config.yaml"


class poseEstimation:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.timestamp = 0

class Kalman_IMU:
    def __init__(self):
        self.K = np.zeros(shape= (2, 2))
        self.xhat=np.zeros(shape= (2, 1))      # a posteri estimate of []
        self.P=np.zeros(shape= (2, 2))         # a posteri error estimate
        self.xhatminus=np.zeros( shape= (2, 1) ) # a priori estimate of x
        self.Pminus=np.zeros( shape= (2, 2) )    # a priori error estimate
        self.z_minus=np.zeros( shape= (2, 1) )
        self.t_minus=0
        self.camDataState = False

class Kalman_Encoder:
    def __init__(self):
        self.K = 0
        self.xhat=0      # a posteri estimate of []
        self.P=0         # a posteri error estimate
        self.xhatminus=0 # a priori estimate of x
        self.Pminus=0    # a priori error estimate
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
        self.yaw = False

class EncoderDisplacement:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.imuYaw = False

def openYamConfig():
    global cam2platform_Rot

    with open(yaml_path, 'r') as stream:
    # check first line to ensure format
        if (stream.readline() != "# VITAG POSITION YAML\n"):
            print " Wrong format! Wrong Vitag position yaml file was selected!"
            exit(0)
        try:
            yaml_obj = yaml.load(stream)
            print "yaml open successfully!"
        except yaml.YAMLError as exc:
            print " Error in reading yaml file!"
            print " Check the format and syntax of yaml"
            exit(0)
    
    cam2platform_Rot = yaml_obj['config']['cam2platform_Rot']
    print "config of cam to platform frame: " , cam2platform_Rot


def plotGraph(time, abs_x, abs_y):
    plt.subplot(2, 1, 1)
    plt.title('x-displacement')
    if plotKalman.newCamData == True:
        plt.plot(time, cam.x, 'r.', markersize=12, label = "vision")
    plt.plot([plotKalman.timeminus, time], [plotKalman.x_minus, abs_x], 'b-', label = "kalman") #line
    
    plt.subplot(2, 1, 2)
    plt.title('y-displacement')
    if plotKalman.newCamData == True:
        plt.plot(time, cam.y, 'c.-', markersize=10, label = "vision")
    plt.plot([plotKalman.timeminus, time], [plotKalman.y_minus, abs_y], 'm-', label = "kalman") #line

    # update for prev for nxt
    plotKalman.x_minus = abs_x
    plotKalman.y_minus = abs_y
    plotKalman.timeminus = time
    plotKalman.newCamData = False        #finsih reading, prepare for new one
    plt.pause(0.01)


def ROS_publishResults(abs_x, abs_y):
    if encoder.imuYaw != False:
        yaw = encoder.imuYaw + cam2platform_Rot  #headings rotation
    elif imu.yaw != False:
        yaw = imu.yaw
    else:
        yaw = cam.yaw
    br.sendTransform((abs_x, abs_y, 0), tf.transformations.quaternion_from_euler(1.571,0, yaw), rospy.Time.now(), '/base_link',"/world")


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


def kalmanFusion_imu(obj, cam_displacement, imu_accel, timestamp):

    state = obj.camDataState
    time_diff = timestamp - obj.t_minus
    obj.t_minus = timestamp
    A_matrix = np.array([[1, time_diff], [0,  1]])
    B_matrix = np.array([[time_diff*time_diff/2], [time_diff]])

    # =================time update (imu dependency) ====================
    obj.xhat[1] = obj.xhat[1] * terminalVel_reduction_factor  #reduce veEncoderDisplacementl every iteration
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
        
    return obj.xhat[0][0]


def kalmanFusion_encoder(obj, cam_displacement, encoder_displacement):
    
    state = obj.camDataState
    # =================time update (encoder dependency) ====================
    obj.xhatminus = obj.xhat + encoder_displacement
    obj.Pminus = obj.P + _Q
    
    # measurement update from camera
    if state == True:
        # ================= measurement update ===========================
        obj.xhat = obj.xhatminus + obj.K * ( cam_displacement - obj.xhatminus)    #equation 18
        obj.K = obj.Pminus / ( obj.Pminus + _R )   #equation 19
        obj.P = (1-obj.K)*obj.Pminus
        #change state to indicate read alr
        obj.camDataState = False
    else:
        obj.xhat = obj.xhatminus #update displacemnt profile when no measurement update

    return obj.xhat


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
        x_fusion_encoder.camDataState = True
        y_fusion_encoder.camDataState = True
        x_fusion_imu.camDataState = True
        y_fusion_imu.camDataState = True
        
        plotKalman.newCamData = True

        print "  (1) Callback from cam!", cam_msg[0], cam_msg[1], cam_msg[2]


def encoder_callback(msg):
    if (time.time() - startTime > 2): #wait all initialize without callback
        encoder_msg = msg.data

        # terbalik according to my room transformation
        encoder.x = encoder_msg[0]
        encoder.y = encoder_msg[1]
        encoder.imuYaw = encoder_msg[2]

        print "  (3) Callback from Encoder!", encoder_msg[0], encoder_msg[1], encoder_msg[2]



if __name__=="__main__":
    openYamConfig() #config param from yaml

    signal.signal(signal.SIGINT, signal_handler)

    cam = poseEstimation()
    plotKalman = plot()

    x_fusion_imu = Kalman_IMU()
    y_fusion_imu = Kalman_IMU()
    
    x_fusion_encoder = Kalman_Encoder()
    y_fusion_encoder = Kalman_Encoder()

    imu = IMUAccel()
    encoder = EncoderDisplacement()

    rospy.init_node("Fusion")
    br = tf.TransformBroadcaster()

    #create call back for both subcriber
    mysub = rospy.Subscriber('cam_poseEstimation', Float32MultiArray, cam_callback)
    # mysub2 = rospy.Subscriber('imu_poseEstimation', Float32MultiArray, imu_callback)
    mysub3 = rospy.Subscriber('encoder_poseEstimation', Float32MultiArray, encoder_callback)

    startTime = time.time()

    plt.figure("Odometry ")
    plt.axis([0, 30, 0, 5])

    while 1:

        timeFromStart = time.time() - startTime
        cam.timestamp= time.time() #NOTE!! need to remove when integration        

        #imu & camera fusion 
        # abs_x = kalmanFusion_imu(x_fusion, cam.x, imu.x_accel, cam.timestamp)
        # abs_y = kalmanFusion_imu(y_fusion, cam.y, imu.y_accel, cam.timestamp)
        
        #encoder &camera fusion
        abs_x = kalmanFusion_encoder(x_fusion_encoder, cam.x, encoder.x) # respect to x
        abs_y = kalmanFusion_encoder(y_fusion_encoder, cam.y, encoder.y)
        
        # === publish results ===
        # plotGraph(timeFromStart, abs_x, abs_y)
        rospy.sleep(0.1)
        ROS_publishResults(abs_x, abs_y)

    rospy.spin()