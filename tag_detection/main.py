import numpy as np
import cv2
import math
import time
import transformation
import matplotlib.pyplot as plt

# ============================ Param =========================================
areaThresh = 100         #area of the square of viTag
circlesNum = 8          #number of circle in viTag
focusLength = 0.7       #in meters (tbc)
cam_matrix = np.matrix( "715.523 0 320;" + 
                        "0 715.523 240;" + 
                        "0 0 1")
dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion
episilon_scale = 1
position_scaleRatio = 0.04026 # pysically measured via camera to convert to cm

# ============================ Param =========================================

startTime = time.time()

#------------- ploting --------------
# plt.axis([0, 50, -90, 90])
# plt.axis([0,100,-2000,2000])
# plt.ion()


def ini():
    #make it global var
    global width, height, cap
    
    #webcam
    cap = cv2.VideoCapture(0)
    width = cap.get(3)
    height = cap.get(4)

    #read from external image
    # frame = cv2.imread('delusional.png',1)
    # frame = cv2.imread('vitag.jpg',1)
    # height, width = frame.shape[:2]

    print("width and height are {} {}".format(width, height))


#invert img
def invert(imagem):
    imagem = (255-imagem)
    return imagem


#Blob Detector Initialize
def blobIni():
    ### Set up the Blob detector with default parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.8

    # Filter by Circularity
    params.filterByConvexity = True
    params.minConvexity = 0.8

    return cv2.SimpleBlobDetector_create(params)

# can use opencv func (cv2.fitline)
#find whether 8 coor or more of the center circles lies on the same line, return bolean 
def lineFitting(keypoints):
    blobNum = len(keypoints)
    if blobNum < 8:         #didnt reach min number of blob for vitag Detection
        return False
    else:
        for i in range(blobNum):
            x = keypoints[i].pt[0]
            y = keypoints[i].pt[1]
            matchCount = 0
            previousGradient = 0
            previousC = 999

            for j in range(1, blobNum):
                x_ = keypoints[i-j].pt[0]
                y_ = keypoints[i-j].pt[1]
                gradient = (y-y_)/(x-x_)
                c = y-(gradient*x)
    
                if (previousGradient*0.92 < abs(gradient) < previousGradient*1.08 and previousC*0.92 < abs(c) < previousC*1.08):
                    matchCount += 1
                    # print ("match!!! matchcount {}".format(matchCount))

                if matchCount == 6:
                    # print("{}, matchcount {} keypoint x {}, y {}, gradient {}, c {} ".format(blobNum, matchCount, x, y, gradient, c))
                    # print("@@@@@@@@@@@@@@@DONE!!!!")

                    return True
                
                # print("{}, matchcount {} keypoint x {}, y {}, gradient {}, c {} ".format(blobNum, matchCount, x, y, gradient, c))
                previousGradient = abs(gradient)
                previousC = abs(c)
            
            # print("nxt pair")
    # time.sleep(5) 
    return False


###  ------------------ not using liao --------------------
#check if point is in contour return bolean
#cast a ray from each point, count num crosses contour's boundary 
def is_Point_inContour(x, y, cnt):
    cross_count = 0
    cnt_list = cnt.tolist()

    while (x< width and y < height):
        if [[x, y]] in cnt_list:
            # print(x, y, " this is match")
            cross_count += 1
        
        # else:
            # print(x, y, "XXXXX")
        x += 1
        y += 1

    #check cross count, odd: in, even: not
    if cross_count%2 == 0:
        # print("cross count false, even {}".format(cross_count))
        return False    #even
    else:
        # print("cross count True, odd {}".format(cross_count))
        return True



#filter undesired contours, return list index of desired contours
def contours_filtering(contours, hierarchy):
    contourIndex_list = []
    
    idx = 0
    if len(contours) != 0:
        for cnt, h in zip(contours, hierarchy[0]):
            area = cv2.contourArea(cnt)
            
            #area filtering
            #add contour larger than certain size to new list
            if (area > areaThresh):
                
                #hierichy filtering
                if h[2] == -1: #find all child contours (with no children) in hiera!
                    # because cnt of parent is same cnt, so need to find grandparents of child(2 hierachy up of the circles cnt)
                    filtered_index = hierarchy[0][h[3]][3]

                    if filtered_index not in contourIndex_list:
                        contourIndex_list.append(filtered_index)

            idx += 1

    return contourIndex_list


    # perspective transformation of viTag Contour
def persTransform(corners_approx, frame):

    # ------------------------- rearrange corners matrix ---------------------
    pts = corners_approx.reshape(4, 2)
    rect = np.zeros((4, 2), dtype = "float32")
    # the top-left point has the smallest sum whereas the
    # bottom-right has the largest sum
    s = pts.sum(axis = 1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    # compute the difference between the points -- the top-right
    # will have the minumum difference and the bottom-left will
    # have the maximum difference
    diff = np.diff(pts, axis = 1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]

    # --------------------- height, width computation ----------------------------
    maxWidth = 400 #ratio is 4:20cm
    maxHeight = 80
    
    # construct our destination points which will be used to
    # map the screen to a top-down, "birds eye" view
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype = "float32")
    
    # calculate the perspective transform matrix and warp
    # the perspective to grab the screen
    homo_Mat = cv2.getPerspectiveTransform(rect, dst)
    # euler = transformation.euler_from_matrix(rotational_Mat)
    # quaternion = transformation.quaternion_from_matrix(rotational_Mat) # to normalize mat, for all ele within -1 to 1
    # euler = transformation.euler_from_quaternion(quaternion)

    # ------------------------ show Cropped viTag Image -----------------------------
    warp = cv2.warpPerspective(frame, homo_Mat, (maxWidth, maxHeight))
    cv2.imshow("percTransform", warp)
    return homo_Mat


#draw final vitag contours
def drawContours(newContours, viTagContours, corners_approx):
    background = np.zeros((height, width,3), np.uint8)
    
    index = -1
    thickness = 1
    color1 = (255, 0, 255)
    color2 = (255, 255, 255)
    cv2.drawContours(background, newContours, index, color1, thickness)  #filtered background contours as pink

    if viTagContours != None:

        cv2.drawContours(background, viTagContours, index, color2, 5)   #vitag box label as white 
        for corners in corners_approx:
            cv2.drawContours(background, corners, -1, (0, 255, 0), 10)
    
    return background


# find the valid 4 sided vitag contour and return vitag contour
def create_viTagContours(contours, contourIndex_list, keypoints ):
    blobNum = len(keypoints)
    viTagContours = []
    viTagContour_size = 999999
    keypointslist_inContour = [] #keypoints coor exist in one contour
    keypointslist_inVitags = []  #confirmed keypoints coor in viTag

    if blobNum < 8:         #didnt reach min number of blob for vitag Detection
        return (None, None)
    
    for idx in contourIndex_list: # loop through each detected contour
        circleTag_count = 0
        cnt = contours[idx]

        for i in range(blobNum): #loop thru circleslabel
            x = keypoints[i].pt[0]
            y = keypoints[i].pt[1]
            
            # if (is_Point_inContour(int(x), int(y), cnt) == True):
            is_inContour = cv2.pointPolygonTest(cnt, (x,y), False)
            if (is_inContour == 1):
                circleTag_count += 1
                keypointslist_inContour.append([x, y])
                # print("cicle count in vitag {}".format(circleTag_count))

        if circleTag_count == circlesNum:
            #update the smaller valid contour as vitag Contour
            # if viTagContour_size > cv2.contourArea(cnt):
            viTagContours.append(cnt)
            keypointslist_inVitags.append(keypointslist_inContour)
        keypointslist_inContour = []

    return (viTagContours, keypointslist_inVitags)
    # return viTagContour

#compute position and pose of the vitag
def positionEstimation(corners_approx, im):
    # ------------------------- rearrange corners matrix ---------------------
    pts = corners_approx.reshape(4, 2)
    image_points = np.zeros((4, 2), dtype = "float32")
    # the top-left point has the smallest sum whereas the
    # bottom-right has the largest sum
    s = pts.sum(axis = 1)
    image_points[0] = pts[np.argmin(s)]
    image_points[2] = pts[np.argmax(s)]
    # compute the difference between the points -- the top-right
    # will have the minumum difference and the bottom-left will
    # have the maximum difference
    diff = np.diff(pts, axis = 1)
    image_points[1] = pts[np.argmin(diff)]
    image_points[3] = pts[np.argmax(diff)]

    # --------------------- height, width computation ----------------------------
    maxWidth = 550 #ratio is 1:5
    maxHeight = 80
    
    # construct our destination points which will be used to
    # map the screen to a top-down, "birds eye" view
    threeD_points = np.array([
        [0, 0, 0],
        [maxWidth - 1, 0, 0],
        [maxWidth - 1, maxHeight - 1, 0],
        [0, maxHeight - 1, 0]], dtype = "float32")

    solvePnp = cv2.solvePnP(threeD_points, image_points, cam_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
    (success, rotation_vector, translation_vector) = solvePnp


    print "Rotation Vector:\n {0}".format(rotation_vector*180/3.14)
    print "Translation Vector:\n {0}".format(translation_vector)
    # print "{0}".format(rotation_vector[1])

    # Project a 3D point (0, 0, 1000.0) onto the image plane.
    # We use this to draw a line sticking out of the nose

    rotation_vector = -rotation_vector

    ## ------------------------ plot on matploty  ------------------------------------
    # currentTime = time.time() - startTime
    # ==> Rotation
    # plt.scatter(currentTime, rotation_vector[0]*180/3.14, color = 'red')    #pitch axis -- havin problem
    # plt.scatter(currentTime,  rotation_vector[1]*180/3.14, color = 'blue')  # yaw axis -- interested
    # plt.scatter(currentTime, rotation_vector[2]*180/3.14, color = 'green')   #row axis
    # ==> Translation
    # plt.scatter(currentTime, translation_vector[0], color = 'red')    #x
    # plt.scatter(currentTime,  translation_vector[1], color = 'blue')  #y
    # plt.scatter(currentTime, translation_vector[2], color = 'green')  #z
    # plt.pause(0.001)

    return (rotation_vector, translation_vector)

#use circle coordinates keypoint to determine the center point of the vitag
def find_viTagCenter(keypoints):
    #method and limitation
    #via poisition x coor, find the 4th and 5th point and average it 
    sortedlist = sorted(keypoints)
    cx = ( sortedlist[3][0] + sortedlist[4][0] )/2
    cy = ( sortedlist[3][1] + sortedlist[4][1] )/2
    return [cx, cy]

#draw the directional pose axis on viTag
def draw_poseaxis(rotation_vector, translation_vector, centerPoints, im, idx):
    
    #params
    axis_length = 200
    cX = int(centerPoints[0])
    cY = int(centerPoints[1])

    #--------------------playing with vectors -------------------------
    yaw_rad = rotation_vector[1][0]
    pitch_rad = rotation_vector[0][0]

    print "viTag {}:: yaw: {}, pitch {} ".format(idx, yaw_rad*180/3.14, pitch_rad*180/3.14)
    x_length = math.sin(yaw_rad)*axis_length
    y_length = math.sin(pitch_rad)*axis_length
    cv2.circle(im, (cX, cY), 3, (0,0,255), -1)

    #------------------- drawing of axis with param ----------------
    p1 = (cX, cY)
    p2 = (int(cX + x_length), int(cY - y_length))
    cv2.line(im, p1, p2, (255,0,0), 5)

    # ------------------ insert text on image -------------------
    #left top corner as reference
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    x= translation_vector[0][0] * position_scaleRatio
    y = translation_vector[1][0] * position_scaleRatio
    z = translation_vector[2][0] * position_scaleRatio
    inputText = "viTag {}:: x: {}cm, y: {}cm, Distance: {}cm".format(idx, int(x), int(y), int(z))
    cv2.putText(im,inputText,(10,400 + idx*35), font, 0.7,(80,80,255),2,cv2.LINE_AA)

    return im

#trackbar change event function
def Trackbar_onChange(trackbarValue):
    global episilon_scale
    episilon_scale = trackbarValue
    return 0

# main code
def main():

    #read from external image
    # frame = cv2.imread('delusional.png',1)
    # frame = cv2.imread('vitag.jpg',1)
    # height, width = frame.shape[:2]
    
    kernel = np.ones((5,5),np.uint8)
    detector = blobIni()
    global episilon_scale

    cv2.namedWindow('Contours')
    cv2.createTrackbar('start','Contours',0, 100, Trackbar_onChange)

    while (True):

        #capture from webcam
        ret, frame = cap.read()
        frame = cv2.resize(frame,(0,0),fx=1,fy=1)

        # ----------------- create suitable binary frame for contour detection ---------------
        
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        # +++++++++++ method1: adaptive Threshold, with dilation and erosion
        # thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 81, 3) #second last odd block size param
        # thresh = cv2.dilate(thresh,kernel,iterations = 1) #dilate
        
        # thresh = cv2.erode(thresh,kernel,iterations = 1) #erode

        # +++++++++++ method 2: canny edge detection
        gray = cv2.bilateralFilter(gray, 11, 17, 17)
        thresh = cv2.Canny(gray, 30, 200) # canny edge detection
        # cv2.imshow("Binary", thresh)


        # ------------------------  Blob Detection ------------------------
        frame = invert(frame)

        ## Detect blobs and find coor
        Blob_keypoints = detector.detect(frame)
        # is_lineFitted = lineFitting(Blob_keypoints)                   #self line fitting method (not in use)
        # print("line fitting results: {}".format(is_lineFitted))

        ## Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        # img_with_keypoints = cv2.drawKeypoints(frame, Blob_keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        ## Show keypoints
        # cv2.imshow("Blob Detection", img_with_keypoints)


        # --------------------- Contour Detection ---------------------
        
        _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contourIndex_list = contours_filtering(contours, hierarchy)

        #create new contours
        newContours = []
        for idx in contourIndex_list:   
            #new contours consist other potential contours besides vitag
            newContours.append(contours[idx])       
        viTagContours, keypointslist_inVitags = create_viTagContours(contours, contourIndex_list, Blob_keypoints)

        # ------------------------- loop through each detected viTag --------------------------
        corners_approx = None
        total_corners_approx = []
        final_image = frame
        if viTagContours != None:
            numContours = len(viTagContours)

            for viTagContour, keypointslist_inVitag, idx in zip(viTagContours, keypointslist_inVitags, range(numContours)):
                # -------------------------- Corners detection ---------------------
                
                #approximat proxy for finding corners      (gonna find a way to limit episilon, or change another way)
                epsilon = 0.02*episilon_scale *cv2.arcLength(viTagContour,True)

                corners_approx = cv2.approxPolyDP(viTagContour,epsilon,True)
                total_corners_approx.append(corners_approx)

                print("Corners")
                print(cv2.minAreaRect(viTagContour))
                print(corners_approx)

                if len(corners_approx) == 4:
                    # draw only when four corners are present
                    print("ViTag Detected: Drawing contour")
                    
                    # -----------------------execution of transformation------------------
                    
                    homo_Mat = persTransform(corners_approx, frame)
                    #smthing = cv2.decomposeHomographyMat(homo_Mat, cam_matrix)
                    rotation_vector, translation_vector = positionEstimation(corners_approx, frame)
                    viTagCenter = find_viTagCenter(keypointslist_inVitag)
                    final_image = draw_poseaxis(rotation_vector, translation_vector, viTagCenter, frame, idx)
            
            print("________________frame______________ ")
        
        background = drawContours(newContours, viTagContours, total_corners_approx)
        cv2.imshow("Contours",background)

        cv2.imshow("Final Output", final_image)


        # # gray = np.float32(gray)
        # dst = cv2.cornerHarris(thresh,23,3,0.04)   #corner harris method, intensive of scanning thru image
        # print(dst)
        # #result is dilated for marking the corners, not important
        # dst = cv2.dilate(dst,None)

        # # Threshold for an optimal value, it may vary depending on the image.
        # frame[dst>0.01*dst.max()]=[0,0,255]

        # cv2.imshow('dst',frame)

        ch = cv2.waitKey(1)

        #break from static
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # break

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