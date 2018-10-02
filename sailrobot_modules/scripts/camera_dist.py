#!/usr/bin/env python
# -*- coding: utf-8 -*-


import cv2

import rospy
#import utilities
from std_msgs.msg import String
import std_msgs
import sensor_msgs
import utilities
import numpy as np
import sys, time

import roslib

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu
from gps_common.msg import GPSFix

from geometry_msgs.msg import Point
import math

import utilities
import rospkg


lat = 0
long = 0
theta = 0
dist_sonde0 = 0
bridge = CvBridge()

"""
liens pour mesurer la distance entre deux coordonnees gps :

https://forums.futura-sciences.com/mathematiques-superieur/520110-calcul-de-distance-coordonnees-gps.html

https://www.01net.com/astuces/astuce-excel-calculez-la-distance-entre-deux-points-de-la-terre-555908.html

https://openclassrooms.com/forum/sujet/calcul-d-une-distance-95555

http://www.movable-type.co.uk/scripts/latlong.html



voir les longitudes :

https://www.latlong.net/lat-long-dms.html

"""





'''
s abonner a la position et inclinaison du bateau, en deduire la pos de l obstacle
publier la pos de l obstacle a l evitement d obstacle

s abonner a /raspicam_node/image/compressed

sensor_msgs/Image.msg

http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber

roslaunch raspicam_node camerav2_1280x720.launch

'''

diametre_connu = 0.66

############################################
#HSV de la sonde et de l aiguille
colorLower_sonde = (18,90,0)  #10,150,60  16, 114, 175
colorUpper_sonde = (35,255,255) #jaune 30,255,255   33, 255, 255
    
############################################


def traitement(frame):
    
    """ Applique un changement de la bande de couleur et une ouverture pour ameliorer la detection de contours"""
    
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    mask_sonde = cv2.inRange(hsv, colorLower_sonde, colorUpper_sonde)
    mask_sonde = cv2.erode(mask_sonde, None, iterations=2)
    mask_sonde = cv2.dilate(mask_sonde, None, iterations=2)
    
    return mask_sonde




def detect_contours(frame, mask_sonde):
    
    """ Detecte et trace les contours """
    
  
    im,contours_sonde, hierarchy = cv2.findContours(mask_sonde,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#    cv2.drawContours(frame, contours_sonde, -1, (0,255,0), 3)
    
    return contours_sonde




def dist_cam_object(frame, contours, n_cam, objet, F=304.0):  #F = 795   304
    
    """ Selectionne le contour le plus grand et l associe a un cercle d un certain rayon puis calcule la distance"""
    
    """file mesure_pixels_230cm.png"""
    # F = distance a l objet en m * diametre en pixel sur l image / taille reelle de l objet en m
    F = 2.30*162/0.66  # default
    F = 4.12*91/0.66   # default
    F = 3.53*127/0.66  # 1280*720
    F = 4.00*52/0.66
    dist = None
    
    if len(contours) > 0:
        cnt = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(cnt)
        center = (int(x),int(y))
        radius = int(radius)
#        print(radius)
#        cv2.circle(frame,center,radius,(0,255,0),2)
#    
        
        #Calcule de la distance camera-aiguille
        dist = diametre_connu * F / (2*radius)
        print("distance camera n_{} - ".format(n_cam) + objet + "= {} m\n".format(dist))

        
    return dist


def coord_distance_gps(lat,long,bearing,d, R=6.371*10**6):
    if(d != None):
        g2 = math.asin( math.sin(lat)*math.cos(d/R) + math.cos(lat)*math.sin(d/R)*math.cos(bearing))
        l2 = long + math.atan2(math.sin(bearing)*math.sin(d/R)*math.cos(lat), math.cos(d/R)-math.sin(lat)*math.sin(g2))
        return g2*180/math.pi, l2*180/math.pi
    return 0,0
    
    

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()




def callback(data):
    global dist_sonde0
    #### direct conversion to CV2 ####
    #np_arr = np.fromstring(data.data, np.uint8)
    frame0 = bridge.imgmsg_to_cv2(data,"bgr8")
    #frame0 = cv2.imdecode(data.data, cv2.IMREAD_COLOR)
    #frame0 = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
    #frame0 = cv2.resize(frame0,(0,0), fx=0.5, fy=0.5)

    mask_sonde0 = traitement(frame0)
    contours_sonde0 = detect_contours(frame0, mask_sonde0)
    dist_sonde0 = dist_cam_object(frame0, contours_sonde0, 0, "sonde")
            
#    cv2.imshow('frame0', frame0)
#    cv2.waitKey(2)
    
    

def callback2(data):
    global theta
    theta = utilities.QuaternionToEuler(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)[2]
    
   
    


def callback3(data):  
    global lat
    global long
    lat = data.latitude
    long = data.longitude
    
def chatter():
    global lat
    global long
    global dist_sonde0
    global theta


    point = Point()
    point.x = 1000
    point.y = 1000

    if (dist_sonde0 < 20)&(dist_sonde0 > 0)&(not(dist_sonde0==None)):

	    rospack = rospkg.RosPack()
	    nomfichier = rospack.get_path('sailrobot') + '/data/coord_Obstacle_dynamic.txt'
 
	#    point = Point()
	#    if(dist_sonde0 == 0.0):
	    coord = coord_distance_gps(lat*math.pi/180,long*math.pi/180,theta*math.pi/180,dist_sonde0, R=6.371*10**6)
	    if(not(coord[0]==0)):
		fichier = open(nomfichier, "w")
	    	point.x = coord[0]
	    	point.y = coord[1]
	#    print((lat,long,theta,dist_sonde0))
	    	fichier.write(str(coord[0]) + "," + str(coord[1]))
	    	fichier.close()

    return point



def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/republished', Image, callback, queue_size=1, buff_size=2**24)
    #rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, callback, queue_size=1, buff_size=2**48)
    rospy.Subscriber('/sailboat/IMU', Imu, callback2)
    rospy.Subscriber('/sailboat/GPS', GPSFix, callback3)
    

    # spin() simply keeps python from exiting until this node is stopped




if __name__ == '__main__':
#       
    try:
        listener()
        pub = rospy.Publisher('/obstacle_coord/', Point, queue_size=10)
#        talker()
        rate = rospy.Rate(1) 
        while not rospy.is_shutdown():
            pub.publish(chatter())
    except rospy.ROSInterruptException:
        print("exception")
        pass 
    
    
#    lat = 50.368925*math.pi/180
#    long = -4.139485*math.pi/180
#    bearing = 0
#    d = 750
#    print(coord_distance_gps(lat,long,bearing,d))
    
    ''' resultats attendus :
    
    50.375932, -4.139614
    
    '''
#    cam0 = cv2.VideoCapture(0)
#
#    
#    while(cam0.isOpened()):
            

#        x,y = gpsMsg.latitude, gpsMsg.longitude
#        xi,yi,zi,wi = imuMsg.orientation.x,imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w
#        theta = utilities.QuaternionToEuler(xi, yi, zi, wi)[2]

#    try:
#        talker()
#    except rospy.ROSInterruptException:
#        pass
