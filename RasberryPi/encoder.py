import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

class Pulse:
    def __init__(self):
        self.state1 = 0
        self.state2 = 0
        self.count = 0
        self.dir = 1

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
            # self.state1 = 0
            self.state2 = 0
        return self.count

pulse = Pulse()
while (1):
    val =  pulse.getNewCount(GPIO.input(4), GPIO.input(17))
    print "encoder: ", val