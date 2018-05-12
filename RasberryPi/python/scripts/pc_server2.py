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


#Global Var
x_accel_list = []
y_accel_list = []
yaw_list = []

# host = '10.27.25.107' #laptop ip
host = '10.27.25.107'#'169.254.228.35'
port1 = 8000
port2 = port1 + 100 
address1 = (host, port1)
address2 = (host, port2)

#ROS
rospy.init_node('imu_publisher_node')
pub_imu=rospy.Publisher('imu_poseEstimation',Float32MultiArray, queue_size = 10)
pub_encoder=rospy.Publisher('encoder_poseEstimation',Float32MultiArray, queue_size = 10)



def plotGraph():
    
    plt.figure("IMU RAW READING ")
    plt.subplot(3, 1, 1)
    plt.title('absX/NS- accel - t')
    plt.plot(x_accel_list, 'r.:')
    plt.ylabel('accel')

    plt.subplot(3, 1, 2)
    plt.title('absY/EW- accel - t')
    plt.plot(y_accel_list, 'c.:')
    plt.ylabel('accel')

    plt.subplot(3, 1, 3)
    plt.title('yaw - t')
    plt.plot(yaw_list, 'r.:')
    plt.ylabel('yaw angle')

    plt.show()
    del x_accel_list[:] #empty list
    del y_accel_list[:]
    del yaw_list[:]


def storePlot(imuData):
    global x_accel_list, y_accel_list, yaw_list

    x_accel_list.append(float(imuData[0])*9.81)
    y_accel_list.append(float(imuData[1])*9.81)
    yaw_list.append(float(imuData[2]))


def ROS_publishResults_IMU(client_data):
    imu = Float32MultiArray()
    NS = float(client_data[1])*9.81
    EW = float(client_data[2])*9.81
    yaw = float(client_data[3])
    time_diff = float(client_data[4])

    imu.data = [NS, EW, yaw, time_diff]
    pub_imu.publish(imu)   


def ROS_publishResults_encoder(client_data):
    encoder = Float32MultiArray()
    encoder.data = [client_data[1], client_data[2]]
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
                # plotGraph() #for plot
                break

            elif output:
                print "Message received from client:"
                print output

                #ouput received readings here!
                if isIMU == True:
                    ROS_publishResults_IMU( output.split(';') )
                else:
                    ROS_publishResults_encoder( output.split(';') )
                    
                #for plotting
                # storePlot(imuData)

                conn.send("ack")


if __name__=="__main__":
    imuThread = threading.Thread(name='imu', args = (address1, '(imu)'), target= serverThread)
    encoderThread = threading.Thread(name='encoder', args = (address2, '(encoder)'), target= serverThread)
    
    imuThread.start()
    encoderThread.start()    
    imuThread.join()
    encoderThread.join()
    
