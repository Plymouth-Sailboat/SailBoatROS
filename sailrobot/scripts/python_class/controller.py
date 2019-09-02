#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from gps_common.msg import GPSFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

import math
import numpy
from enum import Enum
import time
import utilities

class MODE:
	STANDBY, RUDDER_SAIL, RETURN_HOME, HEADING, RC, SAIL_CAP, RUDDER = range(7)

class Controller:
	name = ''
	looprate = 10
	controller = 0

 	windMsg = Pose2D()
 	gpsMsg = GPSFix()
 	imuMsg = Imu()
 	velMsg = Twist()
    sailAngle = 0.0
    rudderAngle = 0.0
    rudder2Angle = 0.0
	windArray = []

	def wakeup(self,event):
        self.publishMSG("C" + str(self.controller))
    def loop(self):
        control = self.control()
        if(control):
            self.publishCMD(control)
        self.publishOdom()

        self.rate.sleep()

    def gps(self,data):
        self.gpsMsg = data

    def imu(self,data):
        self.imuMsg = data

    def wind(self,data):
        self.windMsg = data

#            self.windArray.append(data.theta)
#	    if(len(self.windArray) > 25):
#		del self.windArray[0]
#	    ws = numpy.sum(numpy.sin(self.windArray))
#	    wc = numpy.sum(numpy.cos(self.windArray))
#	    w = math.atan2(ws,wc)
#	    self.windMsg.theta = w
#	    self.windMsg.x = data.x
#	    self.windMsg.y = data.y

    def sail(self,data):
        self.sailAngle = data.data

    def rudder(self,data):
        self.rudderAngle = data.data

    def rudder2(self,data):
        self.rudder2Angle = data.data

    def vel(self,data):
        self.velMsg = data

    def publishOdom(self):
        odom_msg = Odometry();
        odom_msg.pose.pose.position.x = self.gpsMsg.latitude;
        odom_msg.pose.pose.position.y = self.gpsMsg.longitude;
        odom_msg.pose.pose.position.z = self.gpsMsg.altitude;

		odom_msg.pose.pose.orientation = self.imuMsg.orientation;
		odom_msg.twist.twist = self.velMsg;
	    self.odomMsg.publish(odom_msg);

    def publishCMD(self, cmd):
        self.pubCmd.publish(cmd)

    def publishMSG(self, msg):
        self.pubMsg.publish(msg)

    def publishLOG(self, msg):
        self.pubLog.publish(msg)

    def __init__(self, name, looprate, mode):
    	self.name = name
    	self.looprate = looprate
    	self.controller = mode

    	rospy.init_node(self.name,anonymous=True)
    	self.rate = rospy.Rate(self.looprate)

    	rospy.Timer(rospy.Duration(30.0), self.wakeup)

    	self.pubCmd = rospy.Publisher('/sailboat/sailboat_cmd', Twist, queue_size=100)
    	self.pubMsg = rospy.Publisher('/sailboat/sailboat_msg', String, queue_size=10)
    	self.pubLog = rospy.Publisher('/sailboat/pc_log', String, queue_size=10)
    	self.odomMsg = rospy.Publisher('/sailboat/odom', Odometry, queue_size=100)
    	self.gpsSub = rospy.Subscriber('/sailboat/GPS/fix', GPSFix, self.gps)
    	self.imuSub = rospy.Subscriber('/sailboat/IMU', Imu, self.imu)
    	self.windSub = rospy.Subscriber('/sailboat/wind', Pose2D, self.wind)
		self.sailSub = rospy.Subscriber('/sailboat/sail', Float32, self.sail)
    	self.rudderSub = rospy.Subscriber('/sailboat/rudder', Float32, self.rudder)
    	self.rudder2Sub = rospy.Subscriber('/sailboat/rudder2', Float32, self.rudder2)
    	self.velSub = rospy.Subscriber('/sailboat/IMU_Dv', Twist, self.vel)

		if(rospy.get_param("config")):
			utilities.ReadConfig("config/config.txt")
    	time.sleep(1)
    	self.publishMSG("C" + str(mode))
