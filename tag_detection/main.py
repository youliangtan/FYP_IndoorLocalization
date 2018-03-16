import numpy as np
import cv2
import math
import time
import transformation
import matplotlib.pyplot as plt
import yaml

# ============================ Param =========================================
areaThresh = 100         #area of the square of viTag
circlesNum = 8          #number of circle in viTag
focusLength = 0.7       #in meters (tbc)
# cam_matrix = np.matrix( "715.523 0 320;" +  # laptop cam 
#                         "0 715.523 240;" + 
#                         "0 0 1")
cam_matrix = np.matrix( "710.718 0 320;" + # webcam cam 
                        "0 710.718 240;" + 
                        "0 0 1")
dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion
position_scaleRatio = 0.0425 # pysically measured via camera to convert to cm

# Vitag frame for target pers transformation
maxWidth = 400 #ratio is 4:20cm
maxHeight = 80
cropFactor = 0.04 # crop og vitaf edges
croppedWidth = int(maxWidth*cropFactor)
croppedHeight = int(maxHeight*cropFactor)

print( "cropped pixels h {}  w {}".format(croppedHeight, croppedWidth))

# others,(to be optimized)
param1=15
param2=20
z_index = 0
yaml_obj = 0
yaml_path = "markers.yaml"

# ============================ End Param ===================================

startTime = time.time()

#------------- ploting --------------
# plt.axis([0, 50, -90, 90])
# plt.axis([0,100,-2000,2000])
# plt.ion()


def ini():
    #make it global var
    global yaml_obj, width, height, cap
    
    #webcam
    cap = cv2.VideoCapture(0)
    width = int(cap.get(3))
    height = int(cap.get(4))

    #read from external image
    # frame = cv2.imread('delusional.png',1)
    # frame = cv2.imread('vitag.jpg',1)
    # height, width = frame.shape[:2]

    #open yaml file
    with open(yaml_path, 'r') as stream:
        # check first line to ensure format
        if (stream.readline() != "# VITAG POSITION YAML\n"):
            print " Wrong format! Wrong Vitag position yaml file was selected!"
            exit(0)
        try:
            yaml_obj = yaml.load(stream)
            print "yaml open successfully!"
        except yaml.YAMLError as exc:
            print " Error in reading yaml file!"
            exit(0)

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
def persTransform(corners_approx, gray):

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

    # ------------------------  Cropped viTag Image -----------------------------
    viTag_Im = cv2.warpPerspective(gray, homo_Mat, (maxWidth, maxHeight))
    return homo_Mat, viTag_Im


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


    # print "Rotation Vector:\n {0}".format(rotation_vector*180/3.14)
    # print "Translation Vector:\n {0}".format(translation_vector)

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
def draw_poseaxis(rotation_vector, translation_vector, centerPoints, im, markerinfo, idx):
    global z_index

    #params
    axis_length = 200
    cX = int(centerPoints[0])
    cY = int(centerPoints[1])

    #--------------------playing with vectors -------------------------
    yaw_rad = -rotation_vector[1][0] 
    pitch_rad = rotation_vector[0][0]

    # print "viTag {}:: yaw: {}, pitch {} ".format(idx, yaw_rad*180/3.14, pitch_rad*180/3.14)
    x_length = - math.sin(yaw_rad)*axis_length
    y_length = math.sin(pitch_rad)*axis_length
    cv2.circle(im, (cX, cY), 3, (0,0,255), -1)

    #------------------- drawing of axis with param ----------------
    p1 = (cX, cY)
    p2 = (int(cX + x_length), int(cY - y_length))
    cv2.line(im, p1, p2, (255,0,0), 5)

    # ------------------ insert text on image -------------------
    #left top corner as reference
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    x=  translation_vector[0][0] * position_scaleRatio  
    y = translation_vector[1][0] * position_scaleRatio 
    z = translation_vector[2][0] * position_scaleRatio
    
    print " z/sin(yaw)  > {}".format(z*math.sin(yaw_rad))
    print "## {}".format(z*math.sin(yaw_rad) + x)

    # ---------------- compute absolute position ---------------- 
    if markerinfo != None: 
        abs_x = x + markerinfo['x']
        abs_y = y + markerinfo['y']
        abs_z = z + markerinfo['z']
        abs_yaw  = yaw_rad + markerinfo['yaw']
        print "viTag {}: x: {}; y: {}; z: {}; yaw: {}".format(idx, abs_x, abs_y, abs_z , abs_yaw*180/3.14)
    else:
        "invalid marker!!!"
    
    inputText = "viTag {}:: x: {}cm, y: {}cm, Distance: {}cm".format(idx, int(x), int(y), int(z))
    cv2.putText(im,inputText,(10,400 + z_index*35), font, 0.7, (80,80,255) , 2 , cv2.LINE_AA)
    z_index = z_index + 1

    return im


# input vitag im, output index info of the detected ternary marker
def indexFromVitag(viTag_Im):
    
    ternary_str = ""
    index = None
    vitagArray = [] # [[cx0, radius1],[],[]...[cx7, radius7]]

    # ---------------------- circle size detection !! --------------------------
    # crop image edges to ensure clean frame
    viTag_Im = viTag_Im[ croppedHeight : (maxHeight - croppedHeight), croppedWidth : (maxWidth - croppedWidth)]

    viTag_Im = cv2.resize(viTag_Im, (0,0), fx=0.8, fy=0.6) 
    viTag_Im = cv2.medianBlur(viTag_Im,7)
    (thresh, viTag_bw) = cv2.threshold(viTag_Im, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    cv2.imshow("Binary Vitag", viTag_bw)
    
    _, circlesCnt, hierarchy = cv2.findContours(viTag_bw, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if (len(circlesCnt) == 8) : #TODO gonna process if fake contour detected
        for c in circlesCnt:
            cv2.drawContours(viTag_Im, [c], -1, (0, 255, 0), 2)
            # compute the center of the contour
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            # compute area
            area = cv2.contourArea(c)
            vitagArray.append([cX, area])

            # draw the contour and center of the shape on the image
            cv2.circle(viTag_Im, (cX, cY), 5, (0, 255, 0), -1)
            cv2.putText(viTag_Im, "CENTER", (cX - 5, cY - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
    
        vitagArray.sort()  # sort array according to cx
        # print(vitagArray)

        # ------------------------ decode index from size ---------------------------
        # get first tag size as thres
        sizeTresh = vitagArray[0][1]
        smallerSizeTresh = sizeTresh*0.55   #some allowance for treshhold
        largerSizeTresh = sizeTresh*1.6

        for idx in range(1, 8):
            size = vitagArray[idx][1]
            if (size < smallerSizeTresh):
                digit = '0'
            elif (size < largerSizeTresh):
                digit = '1'
            else:
                digit = '2'
            
            ternary_str = ternary_str + digit
        
        index  = int(ternary_str, 3)
        print "results", ternary_str, index
    else:
        print "error in obtaining vitag index" 

    cv2.imshow("percTransform", viTag_Im)

    return index

def getMarkerPositionFromYaml(idx):
    global yaml_obj
    # print yaml_obj
    if idx in yaml_obj["markers"]:
        info = yaml_obj["markers"][idx]
        print info
    else:
        info = None
        print "error in obtaining marker position info on idx {}".format(idx)
    
    return info

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


def getCornersFromContour(cnt):
    background = np.zeros((height, width,1), np.uint8)
    img = np.zeros((height, width,3), np.uint8)

    # create temp image for feature reading
    cv2.drawContours(background, [cnt], contourIdx = -1, color = (255, 0, 255), thickness = 1)
    new_corners = cv2.goodFeaturesToTrack(background, maxCorners= 4, qualityLevel = 0.1, minDistance = 10, blockSize = 10, k=0.4)
    new_corners = new_corners.astype(int) #convert float to int

    ## DECOMMENT THIS TO VISUALIZE BLOCKSIZE ON HARRIS PARAMS
    # dst = cv2.cornerHarris(background, blockSize=param1, 3, k=param2*0.01)
    # dst = cv2.dilate(dst, None)
    # img[dst>0.01*dst.max()] = [0,0,255]
    # cv2.imshow("test harris param", dst)

    # imshow on rgb
    cv2.drawContours(img, [cnt], contourIdx = -1, color = (255, 0, 255), thickness = 1)
    cv2.drawContours(img, new_corners, contourIdx = -1, color = (255, 255, 255), thickness = 5)
    cv2.imshow('corner detection2', img)

    return new_corners    
    

# main code
def main():

    #read from external image
    # frame = cv2.imread('delusional.png',1)
    # frame = cv2.imread('vitag.jpg',1)
    # height, width = frame.shape[:2]
    
    kernel = np.ones((5,5),np.uint8)
    detector = blobIni()
    global z_index, param1, param2

    cv2.namedWindow('Contours')
    cv2.createTrackbar('param1','Contours', 1, 100, Trackbar_onChange1)
    cv2.createTrackbar('param2','Contours',1, 100, Trackbar_onChange2)
    cv2.setTrackbarPos('param1','Contours', param1)
    cv2.setTrackbarPos('param2','Contours', param2)

    print "\n= = = = = = = = = = = = = start program = = = = = = = = = = = = =\n"

    while (True):

        #capture from webcam
        ret, frame = cap.read()
        frame = cv2.resize(frame,(0,0),fx=1,fy=1)
        z_index = 0
        

        # ----------------- create suitable binary frame for contour detection ---------------
        
        gray1 = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # for i in circles[0,:]:
        #     # draw the outer circle
        #     cv2.circle(gray1,(i[0],i[1]),i[2],(0,255,0),2)
        #     # draw the center of the circle
        #     cv2.circle(gray1,(i[0],i[1]),2,(0,0,255),3)

        # cv2.imshow('detected circles',gray1)
        
        # +++++++++++ method1: adaptive Threshold, with dilation and erosion
        # thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 81, 3) #second last odd block size param
        # thresh = cv2.dilate(thresh,kernel,iterations = 1) #dilate
        
        # thresh = cv2.erode(thresh,kernel,iterations = 1) #erode

        # +++++++++++ method 2: canny edge detection
        gray2 = cv2.bilateralFilter(gray1, 11, 17, 17)
        thresh = cv2.Canny(gray2, 30, 200) # canny edge detection
        # cv2.imshow("Binary", thresh)


        # ------------------------  Blob Detection ------------------------
        frame = invert(frame)

        ## Detect blobs and find coor
        Blob_keypoints = detector.detect(frame)
        # is_lineFitted = lineFitting(Blob_keypoints)                   #self line fitting method (not in use)

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

            for viTagContour, keypointslist_inVitag in zip(viTagContours, keypointslist_inVitags):
                # -------------------------- Corners detection ---------------------
                
                # #approximat proxy for finding corners      (gonna find a way to limit episilon, or change another way)
                # epsilon = 0.02*cv2.arcLength(viTagContour,True)
                # corners_approx = cv2.approxPolyDP(viTagContour,epsilon,True)
                # total_corners_approx.append(corners_approx)

                corners_approx = getCornersFromContour(viTagContour)

                if len(corners_approx) == 4:
                    # draw only when four corners are present
                    # -----------------------execution of transformation------------------
                    homo_Mat, viTag_Im = persTransform(corners_approx, gray2)
                    #smthing = cv2.decomposeHomographyMat(homo_Mat, cam_matrix)
                    rotation_vector, translation_vector = positionEstimation(corners_approx, frame)

                    
                    # ----------------- Get Coordinate info from markers --------------
                    # get index from markers
                    index = indexFromVitag(viTag_Im)
                    # info matching from yaml
                    marker_info = getMarkerPositionFromYaml(index)

                    # ------------------ some drawing and output -----------------------
                    viTagCenter = find_viTagCenter(keypointslist_inVitag)
                    final_image = draw_poseaxis(rotation_vector, translation_vector, viTagCenter, frame, marker_info, index)
            
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