import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

count = 0

while (1):
    pulse = GPIO.input(4)
    if pulse != 0:
        print "count: {}, result: {}, {}".format(count, pulse, type(pulse))
    #print "NOW"
    count = count +1
