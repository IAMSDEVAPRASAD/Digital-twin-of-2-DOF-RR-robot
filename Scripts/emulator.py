import numpy as np
import cv2
from cv2 import aruco
import math
from pyzbar import pyzbar
from pyzbar.pyzbar import decode
from  numpy import interp
from zmqRemoteApi import RemoteAPIClient
import zmq

############################# Set link angles ###########################################

def set_values(degree,link):

    if link == 1 :

        link_1 = sim.getObject('/MTB/axis')

        eulerAngles_robot = [0.0, 3.141592502593994,-((degree*(3.141592502593994/180))-3.141592502593994)]
        sim.setObjectOrientation(link_1,sim.handle_parent,eulerAngles_robot)

    if link == 2 :

        link_2 = sim.getObject('/MTB/link/axis')

        eulerAngles_robot = [0.0, 3.141592502593994,((degree*(3.141592502593994/180))-3.141592502593994)]
        sim.setObjectOrientation(link_2,sim.handle_parent,eulerAngles_robot)

############################### Functions for aruco marker ###############################

def centroid(vertexes):
     x_list = vertexes[0][0] + vertexes[0][2] 
     y_list = vertexes[1][0] + vertexes[1][2]
     length = len(vertexes)

     x = x_list / length
     y = y_list / length
     return(int(x), int(y))

def detect_ArUco(img):

    Detected_ArUco_markers = {}
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()
    corners,ids, _ = aruco.detectMarkers(img,aruco_dict,parameters=parameters)

    for i in range(0,ids.shape[0]):
        Detected_ArUco_markers[ids[i][0,]] = corners[i][0,]

    return Detected_ArUco_markers

def Calculate_orientation_in_degree(Detected_ArUco_markers,img):

    ArUco_marker_angles = {}
    cnt = 0
    ans_x = 0
    ans_y = 0
    midpoint_x = 0
    midpoint_y = 0
    right_x = 0
    right_y = 0
    for i in Detected_ArUco_markers:
        for j in Detected_ArUco_markers[i]:
            if (cnt%4) == 0:
                ans_x = j[0,]
                ans_y = j[1,]
                midpoint_x = ans_x
                midpoint_y = ans_y
                cnt = 1
            else:
                cnt += 1
                if cnt == 2:
                    midpoint_x = (midpoint_x + j[0,])/2
                    midpoint_y = (midpoint_y + j[1,])/2
                    right_x = j[0,]
                    right_y = j[1,]
                ans_x += j[0,]
                ans_y += j[1,]
        ans_x = int(ans_x/4)
        ans_y = int(ans_y/4)
        midpoint_x = int(midpoint_x)
        midpoint_y = int(midpoint_y)
        cv2.circle(img,(ans_x,ans_y), 5, (0,0,255), -1)
        cv2.line(img,(ans_x,ans_y),(midpoint_x,midpoint_y),(255,0,0),5)
        midpoint_x = midpoint_x - ans_x
        midpoint_y = -(midpoint_y - ans_y)

        ans_x = 0
        ans_y = 0
        id_str = str(i)
        font = cv2.FONT_HERSHEY_SIMPLEX
        if midpoint_y < 0:
            li = int((360-np.arccos(np.inner([1,0],[midpoint_x,midpoint_y])/np.linalg.norm([midpoint_x,midpoint_y]))*180/np.pi))
            li = li - 90
            
            if li <= 180 :
                li = li
            else:
                li = li - 360
            
            ang = str(li),
            ArUco_marker_angles[i] = ang
        else:
            le = int((np.arccos(np.inner([1,0],[midpoint_x,midpoint_y])/np.linalg.norm([midpoint_x,midpoint_y]))*180/np.pi))
            le = le - 90
            
            if le <= 180 :
                le = le
            else:
                le = -(le-360)
            
            ang = str(le),
            ArUco_marker_angles[i] = ang
    return ArUco_marker_angles

def detect_ArUco_details(image):

 
    ArUco_details_dict = {}
    ArUco_corners = {}
    

    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()
    corners,ids, _ = aruco.detectMarkers(image,aruco_dict,parameters=parameters)

    corner_points = []
    center_points = []


    if len(corners) > 0:

        ids = ids.flatten()

    for (markerCorner, markerID) in zip(corners, ids):

        corners = markerCorner.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners

        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)

        corner_points.append([topLeft, topRight, bottomRight, bottomLeft])
        center_points.append([cX,cY])
        

    ids = ids.tolist()
    Ar_markers = detect_ArUco(image)
    Orientations = Calculate_orientation_in_degree(Ar_markers,image)

    for i in range(0,len(ids)):

        a = Orientations[ids[i]]
        ArUco_details_dict[ids[i]] = [center_points[i],int(a[0])] 

    ArUco_corners = Ar_markers
    
    
    return ArUco_details_dict, ArUco_corners 

def mark_ArUco_image(image,ArUco_details_dict, ArUco_corners):


    for ids, details in ArUco_details_dict.items():
        center = details[0]
        cv2.circle(image, center, 5, (0,0,255), -1)

        corner = ArUco_corners[int(ids)]
        cv2.circle(image, (int(corner[0][0]), int(corner[0][1])), 5, (50, 50, 50), -1)
        cv2.circle(image, (int(corner[1][0]), int(corner[1][1])), 5, (0, 255, 0), -1)
        cv2.circle(image, (int(corner[2][0]), int(corner[2][1])), 5, (128, 0, 255), -1)
        cv2.circle(image, (int(corner[3][0]), int(corner[3][1])), 5, (255, 255, 255), -1)

        tl_tr_center_x = int((corner[0][0] + corner[1][0]) / 2)
        tl_tr_center_y = int((corner[0][1] + corner[1][1]) / 2) 

        cv2.line(image,center,(tl_tr_center_x, tl_tr_center_y),(255,0,0),5)
        display_offset = 2*int(math.sqrt((tl_tr_center_x - center[0])**2+(tl_tr_center_y - center[1])**2))
        cv2.putText(image,str(ids),(center[0]+int(display_offset/2),center[1]),cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
        angle = details[1]
        cv2.putText(image,str(angle),(center[0]-display_offset,center[1]),cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)


    return image

##########################################################################################

if __name__ == "__main__":

    client = RemoteAPIClient()
    sim = client.getObject('sim')

    vid = cv2.VideoCapture(0)

    while(True):

    	ret, frame = vid.read()
    	img = frame.copy()
 
    	set_angle = 0

    	try :

    		### get aruco data ###
    		ArUco_details_dict, ArUco_corners = detect_ArUco_details(img)
    		img = mark_ArUco_image(img, ArUco_details_dict, ArUco_corners)

    		if len(ArUco_details_dict)==2:

    			link_1_angle = ArUco_details_dict[3][1]
    			link_2_angle = ArUco_details_dict[4][1]
    			
    			set_angle = 1 

    			print(link_1_angle,link_2_angle)

    		if set_angle == 1 :

    		    set_values(link_1_angle+180,1)
    		    set_values(link_2_angle,2)


    	except :
    		continue

    	#cv2.imshow('frame', frame)
    	cv2.imshow('aruco', img)

    	if cv2.waitKey(1) & 0xFF == ord('q'):
    		break


    vid.release()
    cv2.destroyAllWindows()
