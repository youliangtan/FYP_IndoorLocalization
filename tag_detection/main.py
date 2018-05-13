import numpy as np
import cv2
import math
import time
import transformation
import matplotlib.pyplot as plt
import yaml
import rospy
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension,Float32MultiArray
# import tf.transformations as tr
import tf


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

# others,(to be optimized)
param1=15               #tracker
param2=30               #tracker
areaThresh = 100        #area of the square of viTag
circlesNum = 8          #number of circle in viTag
yaml_obj = 0
yaml_path = "markers.yaml"
imu_yaw = False

# ============================ End Param ===================================

def imu_callback(msg):
    global imu_yaw
    if (time.time() - startTime > 1): #wait all initialize without callback
        imu_msg = msg.data
        imu_yaw = imu_msg[2] #+ math.pi -0.1 #+ 0.15 #adjust here accorinding to environment
        print "  (2) Callback from IMU!", imu_yaw


br = tf.TransformBroadcaster()
pub=rospy.Publisher('cam_poseEstimation',Float32MultiArray, queue_size = 10)
mysub = rospy.Subscriber('imu_poseEstimation', Float32MultiArray, imu_callback)
startTime = time.time()

#------------- ploting --------------
# plt.axis([0, 50, -90, 90])
# plt.axis([0,100,-2000,2000])
# plt.ion()




def ini():
    #make it global var
    global yaml_obj, width, height, cap, param1, param2

    #ros
    rospy.init_node('cam_publisher_node')
    
    #webcam
    cap = cv2.VideoCapture(0)
    width = int(cap.get(3))
    height = int(cap.get(4))

    #open yaml fileimport numpy as np

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

    # trackbar setup
    cv2.createTrackbar('param1','Final Output', 1, 100, Trackbar_onChange1)
    cv2.createTrackbar('param2','Contours',1, 300, Trackbar_onChange2)
    cv2.setTrackbarPos('param1','Final Output', param1)
    cv2.setTrackbarPos('param2','Contours', param2)


# #trackbar 1 change event function
def Trackbar_onChange1(trackbarValue):
    global param1
    param1 = trackbarValue
    return 0

#trackbar 2 change event function
def Trackbar_onChange2(trackbarValue):
    global param2
    param2 = trackbarValue
    return 0

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

        # filter out if contour is in contour
        return_index_list = list(contourIndex_list)
        for idx in contourIndex_list:
            while hierarchy[0][idx][3] != -1: #loop thru parents of parents till no more ancestor
                idx = hierarchy[0][idx][3]
                if idx in return_index_list:
                    return_index_list.remove(idx)
                    break

    return return_index_list


# draw final vitag contours
def drawNewContours(newContours, viTagContours, corners_approx):
    background = np.zeros((height, width,3), np.uint8)
    index = -1
    thickness = 1
    color1 = (255, 0, 255)
    color2 = (255, 255, 255)
    cv2.drawContours(background, newContours, index, color1, thickness)  #filtered background contours as pink

    if viTagContours != None:
        cv2.drawContours(background, viTagContours, index, color2, 2)   #vitag box label as white 
        for corners in corners_approx:
            cv2.drawContours(background, corners, contourIdx = -1, color = (0, 255, 255), thickness = 10)
    return background


# find the valid 4 sided vitag contour and return vitag contour
# TODO further optimize code
def create_viTagContours(contours, contourIndex_list, keypoints ):
    blobNum = len(keypoints)
    viTagContours = []
    keypointslist_inVitags = []  #confirmed keypoints coor in viTag

    if blobNum < 8:         #didnt reach min number of blob for vitag Detection
        return (None, None)
    
    for idx in contourIndex_list: # loop through each detected contour
        cnt = contours[idx]
        keypointslist_inContour = [] #keypoints coor exist in one contour

        for i in range(blobNum): #loop thru circleslabel
            x = keypoints[i].pt[0]
            y = keypoints[i].pt[1]

            # check if point in contour            
            is_inContour = cv2.pointPolygonTest(cnt, (x,y), False)
            if (is_inContour == 1):
                keypointslist_inContour.append([x, y])

        if len(keypointslist_inContour) == circlesNum:
            viTagContours.append(cnt)
            keypointslist_inVitags.append(keypointslist_inContour)

    return (viTagContours, keypointslist_inVitags)


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
    target_2d = np.array([
            [0, 0],
            [maxWidth - 1, 0],
            [maxWidth - 1, maxHeight - 1],
            [0, maxHeight - 1]], dtype = "float32")
    # calculate the perspective transform matrix and warp
    # the perspective to grab the screen
    homo_Mat = cv2.getPerspectiveTransform(corners_approx, target_2d)
    viTag_Im = cv2.warpPerspective(gray, homo_Mat, (maxWidth, maxHeight))
    return homo_Mat, viTag_Im


# compute position and pose of the vitag
def positionEstimation(corners_approx, im):

    print " =>> Corners height > left {}, right {}".format( corners_approx[3][1] - corners_approx[0][1], corners_approx[2][1] - corners_approx[1][1])
    print " =>> Corners width > left {}, right {}".format( corners_approx[3][0] -  corners_approx[2][0] , corners_approx[0][0] - corners_approx[1][0])
    print " =>> Corners h/w >  {} ".format( (corners_approx[3][0] -  corners_approx[2][0]) / (corners_approx[3][1] - corners_approx[0][1]) )
    # construct our destination points which will be used to
    target_3d = np.array([
        [0, 0, 0],
        [maxWidth - 1, 0, 0],
        [maxWidth - 1, maxHeight - 1, 0],
        [0, maxHeight - 1, 0]], dtype = "float32")
    
    solvePnp = cv2.solvePnP(target_3d, corners_approx, cam_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
    (success, rotation_vector, translation_vector) = solvePnp
    rotation_vector = -rotation_vector

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
def draw_poseaxis(rotation_vector, translation_vector, centerPoints, im, idx, contour_num):

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
    cv2.putText(im,inputText,(10,400 + contour_num*30), font, 0.6, (80,80,255) , 2 , cv2.LINE_AA)
    
    return (im, x, y, z, yaw_rad)



#  compute absolute pose of camera
def getMarkerAbsPose(idx, x, y, z, yaw_rad, markerinfo):

    #incorporation of imu yaw
    print "\nOriginal Yaw: {}, IMU Yaw {}\n".format(yaw_rad, imu_yaw)
    if imu_yaw != False:
        yaw_rad = imu_yaw

    # 2d transformation from cam to vitag
    cam_rotMatrix = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad)], [np.sin(yaw_rad),  np.cos(yaw_rad)]])
    cam_transMatrix = -np.array([[x],[z]])
    cam_tag_transMatrix = -np.matmul(cam_rotMatrix, cam_transMatrix)    

    # 2d transformation from vitag to global origin
    theta = - markerinfo['yaw']
    abs_tag_rotMatrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta),  np.cos(theta)]])
    abs_tag_transMatrix = np.array([[markerinfo['x']],[markerinfo['y']]])
    abs_cam_trans = np.matmul(abs_tag_rotMatrix, cam_tag_transMatrix) + abs_tag_transMatrix

    abs_yaw = theta + yaw_rad

    print "-------------------- Tag {} -------------------".format(idx)
    print "Cam to Tag\t", cam_tag_transMatrix[0], cam_tag_transMatrix[1]
    print "World abs\t", abs_cam_trans[0], abs_cam_trans[1]
    print abs_yaw*180/3.14
    
    return (abs_cam_trans, abs_yaw) 


#size detection on each data circles and return via sorted array
def getDataCirclesArray(viTag_Im):
    vitagArray = [] # [[cx0, radius1],[],[]...[cx7, radius7]]

    # crop image edges to ensure clean frame
    viTag_Im = viTag_Im[ croppedHeight : (maxHeight - croppedHeight), croppedWidth : (maxWidth - croppedWidth)]
    viTag_Im = cv2.resize(viTag_Im, (0,0), fx=0.8, fy=0.8) 
    viTag_Im = cv2.medianBlur(viTag_Im,7)
    (thresh, viTag_bw) = cv2.threshold(viTag_Im, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    _, circlesCnt, hierarchy = cv2.findContours(viTag_bw, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for c in circlesCnt:
        cv2.drawContours(viTag_Im, [c], -1, (0, 255, 0), 2)
        # compute the center of the contour
        M = cv2.moments(c)
        if M["m00"] == 0: #emit zero division error
            return []
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        
        # compute area
        area = cv2.contourArea(c)
        vitagArray.append([cX, area])

        # draw the contour and center of the shape on the image
        cv2.circle(viTag_Im, (cX, cY), 5, (0, 255, 0), -1)
        cv2.putText(viTag_Im, "CENTER", (cX - 5, cY - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)

    cv2.imshow("percTransform", viTag_Im)
    vitagArray.sort()
    return vitagArray  # sort array according to cx


# input vitag im, output index info of the detected ternary marker
def getIndexFromVitag(viTag_Im):
    ternary_str = ""
    vitagArray= getDataCirclesArray(viTag_Im)
        
    if (len(vitagArray) == 8):
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
        return int(ternary_str, 3) #convert index to decimal
    else:
        print "error in obtaining vitag index"
        return None


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
    return new_corners


def ROS_publishResults(x, y, yaw):
    # br.sendTransform((x, y, 0), tf.transformations.quaternion_from_euler(1.571,0, yaw), rospy.Time.now(), '/base_link',"/world")
    a = Float32MultiArray()
    a.data = [x, y, yaw]
    pub.publish(a)    
    

# main code
def main():

    # other setup    
    kernel = np.ones((5,5),np.uint8)
    detector = blobIni()  #blob detector

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

        # ----------------- create binary frame for contour detection via edge detection---------------
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        # gray = cv2.bilateralFilter(gray, 11, 17, 17)
        # thresh = cv2.Canny(gray, param2, 200) # canny edge detection 
        thresh = cv2.Canny(gray, 150, 200) # canny edge detection option 2



        # ------------------------  Blob Detection ------------------------
        invert_frame = invert(frame)

        ## Detect blobs and find coor
        Blob_keypoints = detector.detect(invert_frame)

        # TODO can use opencv func (cv2.fitline)
        # find whether 8 coor or more of the center circles lies on the same line, return bolean 

        # --------------------- Contour Detection ---------------------
        _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contourIndex_list = contours_filtering(contours, hierarchy)


        background2 = np.zeros((height, width,3), np.uint8)
        index = -1
        thickness = 1
        color1 = (255, 0, 255)
        color2 = (255, 255, 255)
        cv2.drawContours(background2, contours, index, color1, thickness)

        cv2.imshow("Raw Contours", background2)



        #create new contours which contists potential contours besides vitag
        newContours = list(map((lambda i: contours[i]), contourIndex_list))
   
        viTagContours, keypointslist_inVitags = create_viTagContours(contours, contourIndex_list, Blob_keypoints)

        # ------------------------- loop through each detected viTag --------------------------
        corners_approx = None
        total_corners_approx = []
        final_image = frame
        if viTagContours != None:
            contour_num = 0
            for viTagContour, keypointslist_inVitag in zip(viTagContours, keypointslist_inVitags):
                # -------------------------- Corners detection ---------------------
                
                corners_approx = getCornersFromContour(viTagContour)
                total_corners_approx.append(corners_approx)

                if len(corners_approx) == 4:
                    # draw only when four corners are present
                    # -----------------------execution of transformation------------------
                    corners_approx = rearrangeCorners(corners_approx)
                    homo_Mat, viTag_Im = persTransform(corners_approx, gray)
                    #smthing = cv2.decomposeHomographyMat(homo_Mat, cam_matrix)
                    rotation_vector, translation_vector = positionEstimation(corners_approx, frame)
                    
                    # ----------------- Get Coordinate info from markers --------------
                    # get index from markers
                    VitagIndex = getIndexFromVitag(viTag_Im)
                    # info matching from yaml
                    marker_info = getMarkerPositionFromYaml(VitagIndex)

                    # ------------------ some drawing and calc of abs pose output -----------------------
                    viTagCenter = getVitagCenter(keypointslist_inVitag)
                    (final_image, x, y, z, yaw_rad) = draw_poseaxis(rotation_vector, translation_vector, viTagCenter, frame, VitagIndex, contour_num)
                    contour_num = contour_num + 1
                    
                    if marker_info != None: #marker exists in yaml
                        abs_cam_trans, abs_yaw = getMarkerAbsPose(VitagIndex, x, y, z, yaw_rad, marker_info) #update global list
                        
                        #update list of coors and yaw
                        vitags_absCoor_list.append(abs_cam_trans)
                        vitags_absYaw_list.append(abs_yaw)

                    else:
                        print "invalid marker for index {}!!!".format(VitagIndex)

                    # marker rel to cam
                    # br.sendTransform((x/100,0,z/100), tf.transformations.quaternion_from_euler(0, 0, yaw_rad),rospy.Time.now(), 'base_link',"world")

            
            # ----------------- calc and show average abs ------------------------        
            if len(vitags_absCoor_list) != 0:
                avgAbsCoor = sum(vitags_absCoor_list)/  len(vitags_absCoor_list)
                avgAbsYaw = sum(vitags_absYaw_list) / len(vitags_absYaw_list)
                inputText = "AVG of {} Tags| x:{:.2f}, y:{:.2f}, yaw {:.2f}".format( 
                    len(vitags_absCoor_list), avgAbsCoor[0][0], avgAbsCoor[1][0], avgAbsYaw*180/3.14)
                print inputText
                cv2.putText(final_image,inputText,(10,20),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,80)  , 2 , cv2.LINE_AA)

                #ros publish to topics
                ROS_publishResults(avgAbsCoor[0][0]/100, avgAbsCoor[1][0]/100, avgAbsYaw)
                

        # ---------------------- cv2 results ------------------------------ 
        hello_str= "________________frame at time %s ______________ " % rospy.get_time()
        print hello_str
        
        background = drawNewContours(newContours, viTagContours, total_corners_approx)
        cv2.imshow("New Contours",background)

        cv2.imshow("Final Output", final_image)
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