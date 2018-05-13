# pc server 2 incorporated both IMU and encoder

    #       ___     MODEL: IMU L3GD20
    #      /00/|                
    #     /0O/ |                  y-axis 
    #    |  |  |__________      /
    #    |  |  /         /|    /===> x-axis
    #    |__| /________ //
    #    |__|__________|/

import socket
import sys
import time
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32MultiArray
import signal
import threading
import numpy as np
import math

#Global Var for plotting
x_accel_list = []
y_accel_list = []
eX_list = []
eY_list = []
odomX_list = [0]
odomY_list = [0]
odomNS_list = [0]
odomEW_list = [0]
yaw_list = []

# param
imu2platform_Rot = math.pi/4

# host = '10.27.25.107' #laptop ip
host = '10.27.25.107'#'169.254.228.35'
port1 = 5000
port2 = port1 + 100 
address1 = (host, port1)
address2 = (host, port2)

#ROS
rospy.init_node('imu_publisher_node')
pub_imu=rospy.Publisher('imu_poseEstimation',Float32MultiArray, queue_size = 10)
pub_encoder=rospy.Publisher('encoder_poseEstimation',Float32MultiArray, queue_size = 10)

class Data:
    def __init__(self):
        self.yaw = 0
        self.imu_NS = 0
        self.imu_EW = 0
        self.encoderX = 0
        self.encoderY = 0
        self.encoder_NS = 0
        self.encoder_EW = 0
        self.time_diff = 0


def plotGraph_imu():
    
    plt.figure("IMU RAW READING ")
    plt.subplot(4, 1, 1)
    plt.title('absX/NS- accel - t')
    plt.plot(x_accel_list, 'r.:')
    plt.ylabel('accel')

    plt.subplot(4, 1, 2)
    plt.title('absY/EW- accel - t')
    plt.plot(y_accel_list, 'c.:')
    plt.ylabel('accel')

    plt.subplot(4, 1, 3)
    plt.title('yaw - t')
    plt.plot(yaw_list, 'r.:')
    plt.ylabel('yaw angle')

    plt.show()
    del x_accel_list[:] #empty list
    del y_accel_list[:]
    del yaw_list[:]


def plotGraph_encoder():
    global eX_list, eY_list, odomNS_list, odomEW_list, odomX_list, odomY_list, yaw_list
    # === figure 1 ====
    plt.figure("Encoder RAW READING ")
    plt.subplot(4, 1, 1)
    plt.title('encoder X - t')
    plt.plot(eX_list, 'r.:')
    plt.ylabel('displacement')

    plt.subplot(4, 1, 2)
    plt.title('encoder Y - t')
    plt.plot(eY_list, 'c.:')
    plt.ylabel('displacement')

    plt.subplot(4, 1, 3)
    plt.title('yaw - t')
    plt.plot(yaw_list, 'r.:')
    plt.ylabel('yaw angle')
    
    plt.subplot(4, 1, 4)
    plt.title('odometry XY - t')
    plt.plot(odomX_list, 'b.:')
    plt.ylabel('meter')
    plt.plot(odomY_list, 'm.:')


    # === figure 2 ====
    plt.figure("Encoder Odometry 2D Map")
    plt.subplot(2, 1, 1)
    plt.title('XY location')
    plt.plot(odomX_list, odomY_list, 'ro')
    # plt.axis([-1.5, 1.5, -1.5, 1.5])
    plt.axis([-3, 3, -3, 3])

    plt.subplot(2, 1, 2)
    plt.title('NSEW location')
    plt.axis([-3, 3, -3, 3])
    plt.plot(odomNS_list, odomEW_list, 'bo')
    
    plt.show()

    del eX_list[:] #empty list
    del eY_list[:]
    odomX_list = [0]
    odomY_list = [0]
    odomNS_list = [0]
    odomEW_list = [0]
    del yaw_list[:]

    

def storePlot_imu():
    global x_accel_list, y_accel_list, yaw_list

    x_accel_list.append(data.imu_NS)
    y_accel_list.append(data.imu_EW)
    yaw_list.append(data.yaw)


def storePlot_encoder():
    global eX_list, eY_list, odomNS_list, odomEW_list, odomX_list, odomY_list, yaw_list
    
    eX_list.append(data.encoderX)
    eY_list.append(data.encoderY)
    odomX_list.append( odomX_list[len(odomX_list) -1 ] + data.encoderX )
    odomY_list.append( odomY_list[len(odomY_list) -1 ] + data.encoderY )
    print "NS!!!!!!:", odomNS_list[len(odomNS_list) -1 ] + data.encoder_NS 
    print "EW!!!!!!", odomEW_list[len(odomEW_list) -1 ] + data.encoder_EW
    print "yaw", data.yaw

    odomNS_list.append( odomNS_list[len(odomNS_list) -1 ] + data.encoder_NS )
    odomEW_list.append( odomEW_list[len(odomEW_list) -1 ] + data.encoder_EW )
    
    yaw_list.append(data.yaw)


# 2d transformation from platfrom XY to NSEW
def encoder_frameTransformation():
    yaw_rad = data.yaw 
    x = data.encoderX
    y = data.encoderY

    platform_rotMatrix = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad)], [np.sin(yaw_rad),  np.cos(yaw_rad)]])
    platform_transMatrix = -np.array([[x],[y]])
    NSEW_transMatrix = -np.matmul(platform_rotMatrix, platform_transMatrix)    
    data.encoder_NS = NSEW_transMatrix[0]
    data.encoder_EW = NSEW_transMatrix[1]
    

def ROS_publishResults_IMU(client_data):
    imu = Float32MultiArray()
    data.imu_NS = float(client_data[1])*9.81
    data.imu_EW = float(client_data[2])*9.81
    data.yaw = float(client_data[3]) + imu2platform_Rot 
    data.time_diff = float(client_data[4])

    imu.data = [data.imu_NS, data.imu_EW, data.yaw, data.time_diff]
    pub_imu.publish(imu)   


def ROS_publishResults_encoder(client_data):
    encoder = Float32MultiArray()
    data.encoderX = float(client_data[1])
    data.encoderY = float(client_data[2])
    
    #transfrom frame from xy to nsew
    encoder_frameTransformation()
    
    encoder.data = [data.encoderX, data.encoderY]
    pub_encoder.publish(encoder)


def serverThread(address, source):
    #ini server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(address)
    server_socket.listen(5)
    
    isIMU = True if source == '(imu)' else False
    time.sleep(1)

    while True:
        print "\n{:5} Listening for client... {}".format(source, address)
        
        conn, address = server_socket.accept()
        print source, "{:5} Connected to client",format(source)
        #pick a large output buffer size because i dont necessarily know how big the incoming packet is                                                    

        while True:
            output = conn.recv(2048);
            if output.strip() == "disconnect":
                conn.close()
                # sys.exit("Received disconnect message.  Shutting down.")
                print "{:5} disconnect current client!".format(source)
                print conn.close()
                time.sleep(1)
                # plotGraph_imu() #for plot
                if isIMU != True: plotGraph_encoder()
                break

            elif output:
                # print "Message received", output

                #ouput received readings here!
                if isIMU == True:
                    ROS_publishResults_IMU( output.split(';') )
                else:
                    ROS_publishResults_encoder( output.split(';') )
                    
                #for plotting
                if isIMU == True:
                    # storePlot_imu()
                    pass
                else:
                    storePlot_encoder()

                conn.send("ack")


if __name__=="__main__":
    data = Data()
    imuThread = threading.Thread(name='imu', args = (address1, '(imu)'), target= serverThread)
    encoderThread = threading.Thread(name='encoder', args = (address2, '(encoder)'), target= serverThread)
    
    imuThread.start()
    encoderThread.start()    
    imuThread.join()
    encoderThread.join()
    
