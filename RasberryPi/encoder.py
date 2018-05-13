#Encoder odometry reading
#wheel radius

import RPi.GPIO as GPIO
import socket
import thread
import time
import signal
import sys

GPIO.setmode(GPIO.BCM)

#odomX
GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

#odomY
GPIO.setup(14, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# param
unit = 0.00157 # m/pulse count 
slipFactor = 1.1


# =========== client-server initialization =================
if True:
    host = '10.27.25.107'#'169.254.228.35' #laptop ip
    port = 8100
    print "Connecting to server"
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((host, port))


class Pulse:
    def __init__(self):
        self.state1 = 0
        self.state2 = 0
        self.count = 0
        self.sentCount = 0
        self.dir = 1

    #update new pulse and get abs count val
    def getNewCount(self, raw1, raw2):
        if raw1 == 1:
            self.state1 = 1
        if raw2 == 1:
            if raw1 == 1:
                self.dir = 1
            else:
                self.dir = -1
            self.state2 = 1

        if (self.state2 and raw2 == 0):
            self.count = self.count + self.dir
            self.state2 = 0

        return self.count

    def getSentDiff(self):
        diff = self.count - self.sentCount
        self.sentCount = self.count
        # convert to diff to cm
        return diff*unit*slipFactor


#send data from client to server in 0.1 interval (according to skip_send)
def sendDataToServer():
    while 1:        
        send_data = "encoder;{};{};".format(encoderX.getSentDiff(), encoderY.getSentDiff())
        print "To Server: ", send_data
        client_socket.send(send_data)

        while client_socket.recv(2048) != "ack":
            print "Failed to connect to server!"
            print "waiting for ack"
            time.sleep(1)

        time.sleep(0.1)


#ctrl-c handler
def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    #send disconnect message
    time.sleep(0.05)                                                                                                                           
    dmsg = "disconnect"
    print "Disconnecting then exit"
    client_socket.send(dmsg)
    sys.exit(0)



#main function
signal.signal(signal.SIGINT, signal_handler)       

# ini encoder
encoderX = Pulse()
encoderY = Pulse()

# Create two threads as follows
try:
    thread.start_new_thread( sendDataToServer, () )
except:
    print "Error: unable to start thread"   


if __name__=="__main__":
    while (1):
        odomX = encoderX.getNewCount(GPIO.input(14), GPIO.input(18))
        odomY = encoderY.getNewCount(GPIO.input(4), GPIO.input(17))
        time.sleep(0.0006)