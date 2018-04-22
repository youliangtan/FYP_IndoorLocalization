import sys, getopt
import numpy as np
sys.path.append('.')
import RTIMU
import os.path
import time
import math
import socket

# ========= global variables =================
prev_timestamp = time.time()*1000000
skip_send = 5

offsetavg_sample_count = 300
offsetavg_x = 0
offsetavg_y = 0

class IMUData:
    def __init__(self):
        self.x_accel = 0
        self.y_accel = 0
        self.yaw = 0
        self.timediff = 0  #total time diff along count
        self.x_accel_sum = 0
        self.y_accel_sum = 0
        self.count = 0


# ========== global imu class delaration ============= 
if True:
    SETTINGS_FILE = "RTIMULib_090518_calib1"

    print("Using settings file " + SETTINGS_FILE + ".ini")
    if not os.path.exists(SETTINGS_FILE + ".ini"):
        print("Settings file does not exist, will be created")

    s = RTIMU.Settings(SETTINGS_FILE)
    imu = RTIMU.RTIMU(s)

    if (not imu.IMUInit()):
        print("IMU Init Failed")
        sys.exit(1)
    else:
        print("IMU Init Succeeded")

    poll_interval = imu.IMUGetPollInterval()
    print("Recommended Poll Interval: %dmS\n" % poll_interval)


# =========== client _ server initialization =================
if True:
    host = '192.168.1.135' #laptop ip
    port = 8000

    print "Connecting to server"
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((host, port))



#initialization
def init():

    global imu, offsetavg_x, offsetavg_y
    sum_x = 0
    sum_y = 0
    count = 0

    # this is a good time to set any fusion parameters
    print("IMU Name: " + imu.IMUName())
    
    imu.setSlerpPower(0.02)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)

    # compute average on accel to obtain offset
    print("computing avg offset.... wait 5s")
    while True:

        if imu.IMURead():
            data = imu.getIMUData()
            raw_x, raw_y, raw_z = data['accel']

            roll = data['fusionPose'][0]
            pitch = data['fusionPose'][1]


			#TODO should compute abs x and y after compute rotational matrix
            abs_x = raw_x*math.cos(pitch) + raw_z*math.sin(pitch)            
            abs_y= raw_y*math.cos(roll) + raw_z*math.sin(roll)
            #print "{} {} {}".format(roll, raw_y*math.cos(roll), raw_z*math.sin(roll))   

            # ignore initial 2s dirty data from imu
            if (time.time() - startTime) > 2:
                if count < offsetavg_sample_count:
                    sum_x = sum_x + abs_x
                    sum_y = sum_y + abs_y
                    count = count+1
                else:
                    break
        
            time.sleep(poll_interval*3.0/1000.0)
    
    offsetavg_x = sum_x/offsetavg_sample_count
    offsetavg_y = sum_y/offsetavg_sample_count
    print " - offsetavg_x is {}".format(offsetavg_x)
    print " - offsetavg_y is {}".format(offsetavg_y)
    print "Start time {}, now {}".format(startTime, time.time())






#send data from client to server in 0.1 interval (according to skip_send)
def sendDataToServer(delta_t):
    
    if (result.count == skip_send): 
        x_avgAccel = result.x_accel_sum / result.count
        y_avgAccel = result.y_accel_sum / result.count

        send_data = "{};{};{};{}".format(x_avgAccel, y_avgAccel, result.yaw, result.timediff)
        print "To Server: ", send_data
        client_socket.send(send_data)
        while client_socket.recv(2048) != "ack":
            print "Failed to connect to server!"
            print "waiting for ack"
            time.sleep(1)
        result.count = 0
        result.timediff = 0

    else:
        result.x_accel_sum = result.x_accel
        result.y_accel_sum = result.y_accel
        result.count = result.count  + 1
        result.timediff = result.timediff + delta_t


    


#main function
if __name__=="__main__":
        
    startTime = time.time()
    init()
    result = IMUData()
    
    print "wait few seconds to kick start!!"    

    while True:
        if imu.IMURead():
            #print "-------- Time stamp: {} ----------".format(time.time())

            # fusion data: means remove gravity on z-axis
            x, y, z = imu.getFusionData()
            #print("FUSION ACCEL:\t %f %f %f" % (x,y,z))

            data = imu.getIMUData()
            raw_x, raw_y, raw_z = data['accel']
            pitch = data['fusionPose'][1]
            roll = data['fusionPose'][0]
            result.yaw = data['fusionPose'][2]

            # compute absolute acceleration on ais parrallel to ground
            result.x_accel = raw_x*math.cos(pitch) + raw_z*math.sin(pitch)
            result.y_accel = raw_y*math.cos(roll) + raw_z*math.sin(roll)

            #compute delta t in terms of seconds
            timestamp = data['timestamp']
            delta_t =  timestamp - prev_timestamp
            delta_t = float(delta_t)/1000000    #0.01s interval 100hz

            #elliminate noisy data during start, waiting....
            if (time.time() - startTime) > 8:            
                sendDataToServer(delta_t)

            prev_timestamp = timestamp
            time.sleep(poll_interval*1.0/1000.0)


























# # result = A_matrix * prev_result + B_matrix * accel
# def odometry_x(accel, duration):
#     global prev_result_x

#     #calib accel
#     accel = accel - offsetavg_x
#     computed_accel = accel* 9.8
   
#     A_matrix = np.array([[1, duration], [0,  1]])
#     B_matrix = np.array([[0.5*duration*duration], [duration]])
#     result = np.matmul( A_matrix, prev_result_x ) + computed_accel * B_matrix

#     prev_result_x = result

#     return prev_result_x[0][0], prev_result_x[1][0], computed_accel



# # result = A_matrix * prev_result + B_matrix * accel
# def odometry_y(accel, duration):
#     global prev_result_y

#     #calib accel
#     accel = accel - offsetavg_y
#     computed_accel = accel* 9.8
   
#     A_matrix = np.array([[1, duration], [0,  1]])
#     B_matrix = np.array([[0.5*duration*duration], [duration]])
#     result = np.matmul( A_matrix, prev_result_y ) + computed_accel * B_matrix

#     prev_result_y = result

#     return prev_result_y[0][0], prev_result_y[1][0], computed_accel