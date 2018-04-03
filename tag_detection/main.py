import numpy as np
import cv2
import math
import time
import transformation
import matplotlib.pyplot as plt
import yaml
import rospy
from std_msgs.msg import String

# ============================ Param =========================================

# cam_matrix = np.matrix( "764.093 0 320;" +  # webcam cam 3 
#                         "0 764.093 240;" + 
                        # "0 0 1")
cam_matrix = np.matrix( "738.811 0 320;" + # webcam cam 
                        "0 738.811 240;" + 
                        "0 0 1")
# cam_matrix = np.matrix( "549.167 0 320;" + # webcam cam 2 
#                         "0 549.167 240;" + 
#                         "0 0 1")                        
dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion
distortion = np.matrix("0.17867552970288433 -1.4352404422882379 0 0 1.9439167199920557")
# distortion = np.matrix("-0.098703031664 0.84872335 0 0 -3.3756656666") # with distortiond
# distortion = np.matrix("-0.0747807 -0.231933 0 0 0.13657657")
position_scaleRatio = 0.036 # pysically measured via camera to convert to cm

# Vitag frame for target pers transformation
maxWidth = 600 #ratio is w:h = 7.6 cm
maxHeight = 80
cropFactor = 0.04 # crop og vitaf edges
croppedWidth = int(maxWidth*cropFactor)
croppedHeight = int(maxHeight*cropFactor)

# all vitag global coor
vitags_absCoor_list = []
vitags_absYaw_list = []

# others,(to be optimized)
param1=15               #tracker
param2=20               #tracker
areaThresh = 100        #area of the square of viTag
circlesNum = 8          #number of circle in viTag
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

    #open yaml file
    print yaml_path
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
            print " Check the format and syntax of yaml"
            exit(0)

    print("width and height are {} {}".format(width, height))

    cv2.namedWindow('Contours')
    cv2.namedWindow("Final Output");
    cv2.moveWindow("Final Output", 80,540); #move win to desired location


#invert img
def invert(imagem):
    imagem = (255-imagem)
    return imagem


#Blob Detector Initialize
def blobIni(): #TODO improvise
    ### Set up the Blob detector with default parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.8

    return cv2.SimpleBlobDetector_create(params)


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


# draw final vitag contours
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
# TODO further optimize code
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

            # check if point in contour            
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


# rearrange corners matrix
def rearrangeCorners(corners_approx):
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

    return image_points


    # perspective transformation of viTag Contour
def persTransform(corners_approx, gray):
    
    # --------------------- height, width computation ----------------------------
    # construct our destination points which will be used to
    # map the screen to a top-down, "birds eye" view
    target = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype = "float32")
    
    # calculate the perspective transform matrix and warp
    # the perspective to grab the screen
    homo_Mat = cv2.getPerspectiveTransform(corners_approx, target)
    # euler = transformation.euler_from_matrix(rotational_Mat)
    # quaternion = transformation.quaternion_from_matrix(rotational_Mat) # to normalize mat, for all ele within -1 to 1
    # euler = transformation.euler_from_quaternion(quaternion)

    # ------------------------  Cropped viTag Image -----------------------------
    viTag_Im = cv2.warpPerspective(gray, homo_Mat, (maxWidth, maxHeight))
    return homo_Mat, viTag_Im


# compute position and pose of the vitag
def positionEstimation(corners_approx, im):

    print " =>> Corners height > left {}, right {}".format( corners_approx[3][1] - corners_approx[0][1], corners_approx[2][1] - corners_approx[1][1])
    print " =>> Corners width > left {}, right {}".format( corners_approx[3][0] -  corners_approx[2][0] , corners_approx[0][0] - corners_approx[1][0])
    print " =>> Corners h/w >  {} ".format( (corners_approx[3][0] -  corners_approx[2][0]) / (corners_approx[3][1] - corners_approx[0][1]) )

    # --------------------- height, width computation ----------------------------
    
    # construct our destination points which will be used to
    # map the screen to a top-down view
    target = np.array([
        [0, 0, 0],
        [maxWidth - 1, 0, 0],
        [maxWidth - 1, maxHeight - 1, 0],
        [0, maxHeight - 1, 0]], dtype = "float32")

    
    solvePnp = cv2.solvePnP(target, corners_approx, cam_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
    (success, rotation_vector, translation_vector) = solvePnp

    rotation_vector = -rotation_vector

    ## ------------------------ plot on matploty  ------------------------------------
    # currentTime = time.time() - startTime
    ## ==> Rotation
    # plt.scatter(currentTime, rotation_vector[0]*180/3.14, color = 'red')    #pitch axis -- havin problem
    # plt.scatter(currentTime,  rotation_vector[1]*180/3.14, color = 'blue')  # yaw axis -- interested
    # plt.scatter(currentTime, rotation_vector[2]*180/3.14, color = 'green')   #row axis
    ## ==> Translation
    # plt.scatter(currentTime, translation_vector[0], color = 'red')    #x
    # plt.scatter(currentTime,  translation_vector[1], color = 'blue')  #y
    # plt.scatter(currentTime, translation_vector[2], color = 'green')  #z
    # plt.pause(0.001)

    return (rotation_vector, translation_vector)


#use circle coordinates keypoint to determine the center point of the vitag
def getVitagCenter (keypoints):
    #method and limitation
    #via poisition x coor, find the 4th and 5th point and average it 
    sortedlist = sorted(keypoints)
    cx = ( sortedlist[3][0] + sortedlist[4][0] )/2
    cy = ( sortedlist[3][1] + sortedlist[4][1] )/2
    return [cx, cy]


#draw the directional pose axis on viTag
def draw_poseaxis(rotation_vector, translation_vector, centerPoints, im, idx):
    global z_index

    #params
    axis_length = 200
    cX = int(centerPoints[0])
    cY = int(centerPoints[1])

    #--------------------playing with vectors -------------------------
    yaw_rad = -rotation_vector[1][0] 
    pitch_rad = rotation_vector[0][0]

    # print "viTag {}:: yaw: {}, pitch {} ".format(idx, yaw_rad*180/3.14, pitch_rad*180/3.14)
    x_length = math.sin(yaw_rad)*axis_length
    y_length = math.sin(pitch_rad)*axis_length
    cv2.circle(im, (cX, cY), 3, (0,0,255), -1)

    #------------------- drawing of axis with param ----------------
    p1 = (cX, cY)
    p2 = (int(cX - x_length), int(cY - y_length))
    cv2.line(im, p1, p2, (255,0,0), 5)

    # ------------------ insert text on image -------------------
    #left top corner as reference
    x = translation_vector[0][0] * position_scaleRatio  
    y = - translation_vector[1][0] * position_scaleRatio 
    z = translation_vector[2][0] * position_scaleRatio
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    inputText = "viTag {}:: x: {}cm, y: {}cm, Distance: {}cm".format(idx, int(x), int(y), int(z))
    cv2.putText(im,inputText,(10,400 + z_index*30), font, 0.6, (80,80,255) , 2 , cv2.LINE_AA)
    z_index = z_index + 1
    
    return (im, x, y, z, yaw_rad)


#  compute absolute pose of camera
def getMarkerAbsPose(idx, x, y, z, yaw_rad, markerinfo):
    global  vitags_absCoor_list, vitags_absYaw_list

    yaw_rad = -yaw_rad ## test
 
    if markerinfo != None: #found marker in yaml
        # 2d transformation from cam to vitag
        cam_rotMatrix = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad)], [np.sin(yaw_rad),  np.cos(yaw_rad)]])
        cam_transMatrix = np.array([[x],[z]])
        cam_tag_transMatrix = np.matmul(cam_rotMatrix, cam_transMatrix)
        
        # 2d transformation from vitag to global origin
        theta = - markerinfo['yaw']
        abs_tag_rotMatrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta),  np.cos(theta)]])
        abs_tag_transMatrix = - np.array([[markerinfo['x']],[markerinfo['y']]])
        abs_cam_trans = np.matmul(abs_tag_rotMatrix, cam_tag_transMatrix) + abs_tag_transMatrix

        abs_yaw = theta + yaw_rad
        print "-------------------- Tag {} -------------------".format(idx)
        print cam_tag_transMatrix
        print abs_cam_trans
        print abs_yaw*180/3.14
        
        #update global list of coors and yaw
        vitags_absCoor_list.append(abs_cam_trans)
        vitags_absYaw_list.append(abs_yaw)
    
    else:
        print "invalid marker for index {}!!!".format(idx)


# input vitag im, output index info of the detected ternary marker
def getIndexFromVitag(viTag_Im):
    
    ternary_str = ""
    index = None
    vitagArray = [] # [[cx0, radius1],[],[]...[cx7, radius7]]

    # ---------------------- circle size detection !! --------------------------
    # crop image edges to ensure clean frame
    viTag_Im = viTag_Im[ croppedHeight : (maxHeight - croppedHeight), croppedWidth : (maxWidth - croppedWidth)]

    viTag_Im = cv2.resize(viTag_Im, (0,0), fx=0.8, fy=0.8) 
    viTag_Im = cv2.medianBlur(viTag_Im,7)
    (thresh, viTag_bw) = cv2.threshold(viTag_Im, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    
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

        index  = int(ternary_str, 3) #converting to decimal
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
    new_corners = cv2.goodFeaturesToTrack(background, maxCorners= 4, qualityLevel = 0.1, minDistance = 10, blockSize = 8, k=0.4)
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
    
    # ros pub setup
    pub = rospy.Publisher('locator', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    # other setup    
    kernel = np.ones((5,5),np.uint8)
    detector = blobIni()  #blob detector
    
    global z_index, param1, param2, vitags_absCoor_list, vitags_absYaw_list

    # trackbar setup
    cv2.createTrackbar('param1','Contours', 1, 100, Trackbar_onChange1)
    cv2.createTrackbar('param2','Contours',1, 100, Trackbar_onChange2)
    cv2.setTrackbarPos('param1','Contours', param1)
    cv2.setTrackbarPos('param2','Contours', param2)

    print "\n= = = = = = = = = = = = = start program = = = = = = = = = = = = =\n"

    while (True):

        #capture from webcam
        ret, frame = cap.read()
        frame = cv2.resize(frame,(0,0),fx=1,fy=1)

        # undistroted distortion from ori frame
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(cam_matrix, distortion, (width,height), 1, (width,height))
        frame = cv2.undistort(frame, cam_matrix, distortion, None, newcameramtx)

        # reiniialize params
        vitags_absCoor_list = []
        vitags_absYaw_list = []
        z_index = 0

        # ----------------- create suitable binary frame for contour detection ---------------
        
        gray1 = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # +++++++++++ method1: adaptive Threshold, with dilation and erosion
        # thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 81, 3) #second last odd block size param
        # thresh = cv2.dilate(thresh,kernel,iterations = 1) #dilate
        # thresh = cv2.erode(thresh,kernel,iterations = 1) #erode

        # +++++++++++ method 2: canny edge detection
        gray2 = cv2.bilateralFilter(gray1, 11, 17, 17)
        thresh = cv2.Canny(gray2, 30, 200) # canny edge detection


        # ------------------------  Blob Detection ------------------------
        invert_frame = invert(frame)

        ## Detect blobs and find coor
        Blob_keypoints = detector.detect(invert_frame)
        # TODO can use opencv func (cv2.fitline)
        # find whether 8 coor or more of the center circles lies on the same line, return bolean 

        ## Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        # img_with_keypoints = cv2.drawKeypoints(frame, Blob_keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
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
                    corners_approx = rearrangeCorners(corners_approx)
                    homo_Mat, viTag_Im = persTransform(corners_approx, gray2)
                    #smthing = cv2.decomposeHomographyMat(homo_Mat, cam_matrix)
                    rotation_vector, translation_vector = positionEstimation(corners_approx, frame)

                    
                    # ----------------- Get Coordinate info from markers --------------
                    # get index from markers
                    index = getIndexFromVitag(viTag_Im)
                    # info matching from yaml
                    marker_info = getMarkerPositionFromYaml(index)

                    # ------------------ some drawing and calc of abs pose output -----------------------
                    viTagCenter = getVitagCenter(keypointslist_inVitag)
                    (final_image, x, y, z, yaw_rad) = draw_poseaxis(rotation_vector, translation_vector, viTagCenter, frame, index)
                    getMarkerAbsPose(idx, x, y, z, yaw_rad, marker_info) #update global list

            
            # ----------------- calc and show average abs ------------------------        
            if len(vitags_absCoor_list) != 0:
                avgAbsCoor = sum(vitags_absCoor_list)/  len(vitags_absCoor_list)
                avgAbsYaw = sum(vitags_absYaw_list) / len(vitags_absYaw_list)
                print " AVERAGE OF {} RESULTS {} {}".format( len(vitags_absCoor_list) , avgAbsCoor, avgAbsYaw)
                
                inputText = "AVG x:{}, y:{}, yaw {}".format(-avgAbsCoor[0][0], avgAbsCoor[1][0], avgAbsYaw*180/3.14)
                cv2.putText(final_image,inputText,(10,20),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,80)  , 2 , cv2.LINE_AA)


        # ---------------------- publishing results ------------------------------ 
        hello_str= "________________frame at time %s ______________ " % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        
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

        if ch & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        # plt.pause(0.005)
        ini()
        main()
    except KeyboardInterrupt:
        print("Shutting down...")
        sys.exit(0)