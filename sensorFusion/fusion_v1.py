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
import thread


# socket server setup
#host = 'localhost'
host = '10.27.198.73' #laptop ip
port = 8800
address = (host, port)
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(address)
server_socket.listen(5)   

# GLOBAL VAR
Q = 0.2**2 # process variance
Q = np.array([[Q, 0], [0, Q]])
R = 0.3**2 # estimate of measurement variance, change to see effect
R = np.array([[R, 0], [0, R ]])
terminalVel_reduction_factor = 0.7

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

class IMUAccel:
    def __init__(self):
        self.x_accel = 0
        self.y_accel = 0


def mytopic_callback(msg):
    cam_msg = msg.data
    
    cam.x = cam_msg[0],
    cam.y = cam_msg[1]
    cam.yaw = cam_msg[2]
    cam.timestamp = time.time()
    cam.newState = True
    print "  Callback from cam!", cam_msg[0], cam_msg[1], cam_msg[2]


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
    obj.xhat[1] = obj.xhat[1] * terminalVel_reduction_factor  #reduce vel every iteration
    obj.xhatminus = np.matmul( A_matrix, obj.xhat ) + imu_accel * B_matrix #equation 7
    obj.Pminus = np.matmul( np.matmul(A_matrix, obj.P) , np.linalg.inv( A_matrix)) + Q    # P = A*P*A^T + Q

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


def IMUReading_server():
    
    while True:
        print "\n SERVER THREAD: Listening for client . . .\n\n"

        conn, address = server_socket.accept()
        print "Connected to client at ", address
        #pick a large output buffer size because i dont necessarily know how big the incoming packet is                                                    

        while True:
            output = conn.recv(2048);
            if output.strip() == "disconnect":
                conn.close()
                # sys.exit("Received disconnect message.  Shutting down.")
                print "disconnect current client!"
                print conn.close()
                time.sleep(1)
                break

            elif output:
                print "Message received from client:"
                
                #ouput received readings here!
                print output
                imuData  = output.split(';')

                # terbalik according to my room transformation
                imu.x_accel = -float(imuData[1])
                imu.y_accel = float(imuData[0])
                
                conn.send("ack")


if __name__=="__main__":

    signal.signal(signal.SIGINT, signal_handler)

    cam = poseEstimation()
    plotKalman = plot()

    x_fusion = Kalman()
    y_fusion = Kalman()
    imu = IMUAccel()

    rospy.init_node("Fusion")
    br = tf.TransformBroadcaster()
    mysub = rospy.Subscriber('cam_poseEstimation', Float32MultiArray, mytopic_callback)
    startTime = time.time()

    plt.figure("Odometry ")
    plt.axis([0, 30, 0, 5])


    # Create thread for IMU measurements time update
    try:
        thread.start_new_thread(IMUReading_server, ())
    except:
        print "Error: unable to start thread"
        exit(0)

    while 1:

        timeFromStart = time.time() - startTime
        cam.timestamp= time.time() #NOTE!! need to remove when integration
        
        kalmanFusion(x_fusion, cam.x, imu.x_accel, cam.timestamp, cam.newState)
        kalmanFusion(y_fusion, cam.y, imu.x_accel, cam.timestamp, cam.newState)

        # print "xhat: : ", x_fusion.xhat.transpose()
        # print "zminus: ", x_fusion.z_minus.transpose()
        # print "From Main {} {} {} | {}".format(cam.x, cam.y, cam.yaw, timeFromStart)
        
        # === publish results ===
        # plotGraph(timeFromStart, x_fusion, y_fusion)
        ROS_publishResults()
        rospy.sleep(0.05)

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