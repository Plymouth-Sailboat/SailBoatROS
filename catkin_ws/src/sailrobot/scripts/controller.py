#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from gps_common.msg import GPSFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

from enum import Enum
import time

class MODE:
	STANDBY, RUDDER_SAIL, RETURN_HOME, HEADING, WAYPOINTS = range(5)

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

	def wakeup(self):
            self.publishMSG("M")
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
		
        def sail(self,data):
            self.sailAngle = data.data
		
        def rudder(self,data):
            self.rudderAngle = data.data
		
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

    	def __init__(self, name, looprate, mode):
        	self.name = name
        	self.looprate = looprate
        	self.controller = mode

        	rospy.init_node(self.name,anonymous=True)
        	self.rate = rospy.Rate(self.looprate)

        	rospy.Timer(rospy.Duration(30.0), self.wakeup)

        	self.pubCmd = rospy.Publisher('/sailboat/sailboat_cmd', Twist, queue_size=100)  
        	self.pubMsg = rospy.Publisher('/sailboat/sailboat_msg', String, queue_size=10)  
        	self.odomMsg = rospy.Publisher('/sailboat/odom', Odometry, queue_size=100)
        	self.gpsSub = rospy.Subscriber('/sailboat/GPS', GPSFix, self.gps)
        	self.imuSub = rospy.Subscriber('/sailboat/IMU', Imu, self.imu)
        	self.windSub = rospy.Subscriber('/sailboat/wind', Pose2D, self.wind)
		self.sailSub = rospy.Subscriber('/sailboat/sail', Float32, self.sail)
        	self.rudderSub = rospy.Subscriber('/sailboat/rudder', Float32, self.rudder)
        	self.velSub = rospy.Subscriber('/sailboat/IMU_Dv', Twist, self.vel)

        	time.sleep(1)
        	self.publishMSG("C" + str(mode))
