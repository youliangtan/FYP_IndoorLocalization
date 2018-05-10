import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

count = 0

class Pulse:
    def __init__(self):
        self.state = 0
        self.count = 0

    def getNewCount(self, raw):
        if raw == 1:
            self.state = 1
        if (self.state and raw == 0):
            self.count = self.count + 1
        return count

pulse = Pulse()
while (1):
    val =  pulse.getNewCount(GPIO.input(4))
    print "encoder: ", val