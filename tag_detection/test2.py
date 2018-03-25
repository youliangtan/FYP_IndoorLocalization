import numpy as np
import cv2
import math
import time
import transformation
import matplotlib.pyplot as plt

param1= 100
param2 = 100


def ini():
    #make it global var
    global width, height, cap
    
    #webcam
    cap = cv2.VideoCapture(0)
    width = int(cap.get(3))
    height = int(cap.get(4))

    #read from external image
    # frame = cv2.imread('delusional.png',1)
    # frame = cv2.imread('vitag.jpg',1)
    # height, width = frame.shape[:2]

    print("width and height are {} {}".format(width, height))


#trackbar 1 change event function
def Trackbar_onChange1(trackbarValue):
    global param1
    param1 = trackbarValue
    return 0

#trackbar 2 change event function
def Trackbar_onChange2(trackbarValue):
    global param2
    param2 = trackbarValue
    return 0


# main code
def main():
    
    kernel = np.ones((5,5),np.uint8)
    global param1, param2


    cv2.namedWindow('detected circles')
    cv2.createTrackbar('param1','detected circles',0, 200, Trackbar_onChange1)
    cv2.createTrackbar('param2','detected circles',0, 200, Trackbar_onChange2)
    cv2.setTrackbarPos('param1','detected circles', param1)
    cv2.setTrackbarPos('param2','detected circles', param2)

    while (True):

        #capture from webcam
        ret, frame = cap.read()
        frame = cv2.resize(frame,(0,0),fx=1,fy=1)

        # img = cv2.medianBlur(frame,5)
        # cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

        cimg = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)


        #hough circle method!!
        circles = 0;
        circles = cv2.HoughCircles(cimg,cv2.HOUGH_GRADIENT,1,20,param1 = param1, param2 = param2,minRadius=0,maxRadius=50)
        # print(circles.size)
        circlesprintout = "{}".format(circles)  #some noob None readable error, thus using this
        print(circlesprintout)
        if (circlesprintout[0] != "N"):

            print(len(circles[0]))
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                # draw the outer circle
                cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
        else:
            print("NONE")
        
        cv2.imshow('detected circles',cimg)
        
        ch = cv2.waitKey(1)
        if ch & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        plt.pause(0.005)
        ini()
        print("width: {}, height: {}".format(width, height))
        main()
    except KeyboardInterrupt:
        print("Shutting down...")
        sys.exit(0)