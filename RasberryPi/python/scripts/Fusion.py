import sys, getopt
import numpy as np
sys.path.append('.')
import RTIMU
import os.path
import time
import math


def init():
    SETTINGS_FILE = "RTIMULib_calibrated"

    print("Using settings file " + SETTINGS_FILE + ".ini")
    if not os.path.exists(SETTINGS_FILE + ".ini"):
    print("Settings file does not exist, will be created")

    s = RTIMU.Settings(SETTINGS_FILE)
    imu = RTIMU.RTIMU(s)

    print("IMU Name: " + imu.IMUName())

    if (not imu.IMUInit()):
        print("IMU Init Failed")
        sys.exit(1)
    else:
        print("IMU Init Succeeded")

    # this is a good time to set any fusion parameters

    imu.setSlerpPower(0.02)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)


def main():
    poll_interval = imu.IMUGetPollInterval()
    print("Recommended Poll Interval: %dmS\n" % poll_interval)

    while True:
        if imu.IMURead():
            print "-------- Time stamp: {} ----------".format(time.time())

            # fusion data: means remove gravity on z-axis
            x, y, z = imu.getFusionData()
            print("FUSION ACCEL:\t %f %f %f" % (x,y,z))


            data = imu.getIMUData()
            raw_x, raw_y, raw_z = data['accel']
            print("RAW ACCEL:\t %f %f %f" % (raw_x, raw_y, raw_z))
            print("ABS ACCEL:\t {}".format(math.sqrt(raw_x*raw_x + raw_y*raw_y + raw_z*raw_z)))


            fusionPose = data["fusionPose"]
            print("ROTATION:\t %f p: %f y: %f" % (math.degrees(fusionPose[0]), math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
            time.sleep(poll_interval*10.0/1000.0)



if __name__ == '__main__':
    try:
        # plt.pause(0.005)
        init()
        main()
    except KeyboardInterrupt:
        print("Shutting down...")
        sys.exit(0)