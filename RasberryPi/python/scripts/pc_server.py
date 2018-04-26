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

#Global Var
x_accel_list = []
y_accel_list = []
yaw_list = []

#host = 'localhost'
host = '10.27.198.73' #laptop ip
port = 8800
address = (host, port)

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(address)
server_socket.listen(5)

#ROS
rospy.init_node('imu_publisher_node')
pub=rospy.Publisher('imu_poseEstimation',Float32MultiArray, queue_size = 10)


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

    x_accel_list.append(float(imuData[0]))
    y_accel_list.append(float(imuData[1]))
    yaw_list.append(float(imuData[2]))


def ROS_publishResults(NS, EW, yaw):
    a = Float32MultiArray()
    a.data = [NS, EW, yaw]
    pub.publish(a)   
    

if __name__=="__main__":
    
    while True:
        print "\nListening for client . . ."

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
                plotGraph() #for plot
                break

            elif output:
                print "Message received from client:"
                
                #ouput received readings here!
                print output
                imuData  = output.split(';')

                #for plotting
                storePlot(imuData)
                ROS_publishResults(float(imuData[0]), float(imuData[1]), float(imuData[2]))

                conn.send("ack")