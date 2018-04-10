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
prev_vel = 0
prev_dis = 0
prev_result = np.array([[prev_dis], [prev_vel]])
skip_print = 100
skip_count = 0

# ========== global imu class delaration ============= 
SETTINGS_FILE = "RTIMULib_calibrated"

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

# =========== client _ server initialization =================
host = 'localhost'
port = 8000

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((host, port))




#initialization
def init():
    global imu

    # this is a good time to set any fusion parameters
    print("IMU Name: " + imu.IMUName())
    
    imu.setSlerpPower(0.02)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)

 
#main
def main():
    global imu

    poll_interval = imu.IMUGetPollInterval()
    print("Recommended Poll Interval: %dmS\n" % poll_interval)

    while True:
        if imu.IMURead():
            #print "-------- Time stamp: {} ----------".format(time.time())

            # fusion data: means remove gravity on z-axis
            x, y, z = imu.getFusionData()
            #print("FUSION ACCEL:\t %f %f %f" % (x,y,z))

            data = imu.getIMUData()
            raw_x, raw_y, raw_z = data['accel']
            #print("RAW ACCEL:\t %f %f %f" % (raw_x, raw_y, raw_z))
            #print("ABS ACCEL:\t {}".format(math.sqrt(raw_x*raw_x + raw_y*raw_y + raw_z*raw_z)))
            
            d, v = odometry_x(raw_y, data['timestamp'])
            #print d, v

            fusionPose = data["fusionPose"]
            #print("ROTATION:\t %f p: %f y: %f" % (math.degrees(fusionPose[0]), math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
            
            
            time.sleep(poll_interval*1.0/1000.0)


# result = A_matrix * prev_result + B_matrix * accel
def odometry_x(accel, timestamp):
    global prev_result, prev_timestamp, skip_count

    delta_t = timestamp - prev_timestamp

    #calib accel

    accel = accel* 9.8
    accel = accel + 0.08

    delta_t = float(delta_t)/1000000 
    
    A_matrix = np.array([[1, delta_t], [0,  1]])
    B_matrix = np.array([[0.5*delta_t*delta_t], [delta_t]])
    result = np.matmul( A_matrix, prev_result ) + accel * B_matrix

    prev_result = result
    prev_timestamp = timestamp

    if (skip_count == skip_print): 
        print "test, accel {}, timestamp {}, prev ts {}".format( accel, timestamp, prev_timestamp )
        print "delta t ", delta_t
        print result
        skip_count = 0
    else:
        skip_count = skip_count + 1


    #send results to server
    client_socket.send(data)
    while client_socket.recv(2048) != "ack":
        print "failed to fer server!"
        print "waiting for ack"
        time.sleep(1)


    return result[0], result[1]



if __name__ == '__main__':
    try:
        # plt.pause(0.005)
        init()
        main()
    except KeyboardInterrupt:
        print("Shutting down...")
        sys.exit(0)
