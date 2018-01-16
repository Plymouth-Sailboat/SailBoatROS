#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose2D

class Controller:
	name = ''
	looprate = 10
	controller = 0

	windData = Pose2D()
	gpsData = NavSatFix()
	imuData = Imu()

	def __init__(self, name, looprate, index):
		self.name = name
		self.looprate = looprate
		self.controller = index

		self.pubCmd = rospy.Publisher('sailboat/sailboat_cmd', Twist, queue_size=100)
		self.pubMsg = rospy.Publisher('sailboat/sailboat_msg', String, queue_size=10)
		self.gpsSub = rospy.Subscriber('sailboat/GPS', NavSatFix, self.gps)
		self.imuSub = rospy.Subscriber('sailboat/IMU', Imu, self.imu)
		self.windSub = rospy.Subscriber('sailboat/wind', Pose2D, self.wind)

	def init(self):
		rospy.init_node(self.name,anonymous=True)
		self.rate = rospy.Rate(self.looprate)

	def loop(self):
		self.control()
		self.rate.sleep()
	def gps(self,data):
		self.gpsData = data
	def imu(self,data):
		self.imuData = data
	def wind(self,data):
		self.windData = data
	def publishCMD(self, cmd):
		self.pubCmd.publish(cmd)
	def publishMSG(self, msg):
		self.pubMsg.publish(msg)

