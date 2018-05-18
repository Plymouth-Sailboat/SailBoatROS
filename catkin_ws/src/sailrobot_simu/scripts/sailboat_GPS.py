#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, Pose2D
from gps_common.msg import GPSFix
from sensor_msgs.msg import Imu
from math import pi
import math
from matplotlib.pyplot import *
from numpy import array
import numpy as np
from roblib2 import *
import utilities

from parameters import *


Lx = []
Ly = []


class msg():

	def __init__(self):
		self.x = 0.0
		self.y = 0.0


def callback(data):
	msg1.x = data.angular.x
	msg1.y = data.angular.y


def listener_input():
	sub = rospy.Subscriber('/sailboat/sailboat_cmd',Twist,callback)


def talker_GPS(hello):
	pub = rospy.Publisher('/sailboat/GPS',GPSFix,queue_size=10)
	pub.publish(hello)


def talker_IMU(hello):
	pub = rospy.Publisher('/sailboat/IMU',Imu,queue_size=10)
	pub.publish(hello)


def talker_Wind(hello):
	pub = rospy.Publisher('/sailboat/wind',Pose2D,queue_size=10)
	pub.publish(hello)


def talker_rudder_sail(helloRudder,helloSail):
	pub1 = rospy.Publisher('/sailboat/rudder',Float32,queue_size=10)
	pub1.publish(helloRudder)
	pub2 = rospy.Publisher('/sailboat/sail',Float32,queue_size=10)
	pub2.publish(helloSail)


def talker_velMsg(hello):
	pub = rospy.Publisher('/sailboat/IMU_Dv',Twist,queue_size=10)
	pub.publish(hello)


def plot_sailboat(Lx,Ly,x,y,theta,deltas,deltar,psy_tw,aaw,display_obj,LObj,display_obs,LObs):
        cla()
        plot(Lx,Ly,'b')
# 	 plot(0,0,'*b')


        if display_obs == True:
                for i in arange(0,len(LObs)-1,2):
			for j in range(0,2):
				for k in range(0,2):
                        		dst1 = utilities.GPSDist(LObs[i+j][0],LObs[i+k][1],latref,longref)
                        		theta10 = utilities.GPSBearing(latref,longref,LObs[i+j][0],LObs[i+k][1])
                        		x10 = np.cos(theta10)*dst1
                        		y10 = np.sin(theta10)*dst1
                        		plot(x10,y10,'*r')


	if display_obj == True:
		for i in range(0,len(LObj)):
	                dst1 = utilities.GPSDist(LObj[i][0],LObj[i][1],latref,longref)
                	theta10 = utilities.GPSBearing(latref,longref,LObj[i][0],LObj[i][1])
                	x10 = np.cos(theta10)*dst1
                	y10 = np.sin(theta10)*dst1
			plot(x10,y10,'*b')


        x11 = array([x,y,theta])
        draw_sailboat(x11,deltas,deltar,psy_tw,aaw,'r',5)

        axis((-400, 400, -400, 400))
        draw()


def EulerQuaternion(pitch,roll,yaw):

        cy = np.cos(yaw/2)
        cr = np.cos(roll/2)
        cp = np.cos(pitch/2)

        sy = np.sin(yaw/2)
        sr = np.sin(roll/2)
        sp = np.sin(pitch/2)

        w = cy*cr*cp + sy*sr*sp
        x = cy*sr*cp - sy*cr*sp
        y = cy*cr*sp + sy*sr*cp
        z = sy*cr*cp - cy*sr*sp

        return x, y, z, w


def CartesienToGPS(lat1,long1,x0,y0,x,y):

	Rearth = 6371000

	lat2 = lat1 + 180/pi*(x - x0)/Rearth
	long2 = long1 + 180/pi*(y - y0)/(Rearth*cos((lat1+lat2)/2))*0.5

	return lat2,long2


class Sailboat:

    def __init__(self,a,b,c,psi_tw,atw,dt):

	self.lat = a
	self.long = b

        self.x = 0.0
        self.y = 0.0

        self.theta = c
        self.v = 0.0
        self.u = 0.0
        self.deltas = 0.0
        self.deltar = 0.0
        self.omega = 0.0
        self.dv = 0.0
        self.du = 0.0

        self.psi = 0.0
        self.dx = 0.0
        self.dy = 0.0
        self.dt = dt

        self.psi_tw = psi_tw
        self.atw = atw

        self.psi_aw = psi_tw
        self.aaw = atw



    def dynamic_sailboat(self,ux,uy):


#        p1 = 0.03
#        p2 = 40.0
#        p3 = 6000.0
#        p4 = 200.0
#        p5 = 1500.0
#        p6 = 0.5
#        p7 = 0.5
#        p8 = 2.0
#        p9 = 300
#        p10 = 400
#        p11 = 0.2
#        p100 = p1


        self.deltas = ux # self.deltas - self.dt*(self.deltas - ux)
        self.deltar = uy # self.deltar - self.dt*(self.deltar - uy)


        # vent
        Wcaw = array([self.atw*np.cos(self.psi_tw-self.theta)-self.v*np.sign(self.v) , self.atw*np.sin(self.psi_tw-self.theta)])
        self.aaw = (Wcaw[0]**2 + Wcaw[1]**2)**(0.5)
        self.psi_aw = math.atan2(Wcaw[1],Wcaw[0])

        self.deltas = - np.sign(self.psi_aw)*np.min([np.abs(self.deltas),np.abs(pi-np.abs(self.psi_aw))])


        # vecteur g:
        gs = p4*self.aaw*np.sin(self.deltas - self.psi_aw)
        gr = p5*(self.v**2)*np.sin(self.deltar)
        gru = p5*(self.u**2)*np.cos(self.deltar)


        # system dynamique
        self.dv = (gs*np.sin(self.deltas) - gr*p11*np.sin(self.deltar)-p2*self.v**2*np.sign(self.v) + p100*self.atw**(2)*np.cos(self.psi_tw-self.theta))/p9
        self.du = (- gru*p11*np.cos(self.deltar)*np.sign(self.u) -p2*self.u**2*np.sign(self.u) + p100*self.atw**(2)*np.sin(psi_tw-self.theta))/p9
        domega = (gs*(p6-p7*np.cos(self.deltas)) - gr*p8*np.cos(self.deltar) - p3*self.omega*np.abs(self.v))/p10


        self.v = self.dv*self.dt + self.v
        self.u = self.du*self.dt + self.u

        self.omega = domega*self.dt + self.omega
        self.theta = self.omega*self.dt + self.theta

        self.theta = np.mod(self.theta,2*pi)

        if self.theta > pi:
            self.theta = self.theta - 2*pi

        # calcul de la position
	self.dx = self.v*np.cos(self.theta) -  self.u*np.sin(self.theta)
	self.dy = self.v*np.sin(self.theta) + self.u*cos(self.theta)
        self.x = self.x + self.dt*self.dx
        self.y = self.y + self.dt*self.dy


if __name__ == '__main__':

	# default values
	lat0 = 50.37228  - (50.3727) #(50.3759061)
	long0 = -4.137886200000025  - (-4.13793) #(-4.139577700000018)
        theta0 = 0
	atw = 10.0
	psi_tw =  pi/2.0
        latref = 0.0
        longref = 0.0
	display = False
	display_obj = False
	display_obs = False

        fileGPS = '/home/sailboat/git/SailBoatROS/catkin_ws/src/sailrobot/scripts/coord_GPS.txt'
	fileObs = '/home/sailboat/git/SailBoatROS/catkin_ws/src/sailrobot/scripts/coord_Obstacle.txt'
	LObj = utilities.readGPSCoordinates(fileGPS)
	LObs = utilities.readGPSCoordinates(fileObs)



	# update values
	for i in range(0,len(sys.argv)):

                if sys.argv[i] == '-latrf':
                        latref = float(sys.argv[i+1])

		elif sys.argv[i] == '-lat':
			lat0 = float(sys.argv[i+1])


                if sys.argv[i] == '-longrf':
                        longref = float(sys.argv[i+1])

		elif sys.argv[i] == '-long':
			long0 = float(sys.argv[i+1])


                if sys.argv[i] == '-head':
                        theta0 = float(sys.argv[i+1])*pi/180.0

                if sys.argv[i] == '-tw':
                        psi_tw = float(sys.argv[i+1])*pi/180.0

                if sys.argv[i] == '-atw':
                        atw = float(sys.argv[i+1])


		if sys.argv[i] == '-vobj':
			display_obj = True
			if sys.argv[i+1]=='0':
				print('default path used')
			else:
				fileGPS = sys.argv[i+1]
        			LObj = utilities.readGPSCoordinates(fileGPS)
				if len(LObj) == 0:
					print('No GPS data found')

                if sys.argv[i] == '-vobs':
                        display_obs = True
                        if sys.argv[i+1]=='0':
                                print('default path used')
                        else:
                                fileObs = sys.argv[i+1]
                                LObs = utilities.readGPSCoordinates(fileObs)
                                if len(LObs) == 0:
                                        print('No GPS data found')



                elif sys.argv[i] == '-v':
                        display = True



        print(' -lat : Latitude (current: ',lat0,')' )
        print(' -long : Longitude (current: ',long0,')' )
        print(' -head : Heading (current: ',theta0*180/pi,'(deg))' )
        print(' -tw : true wind orientation (current: ',psi_tw*180/pi,'(deg))' )
        print(' -atw : true wind veloctity (current: ',atw,'(m/s))' )
	print(' -latrf : Latitude reference (for plot. current: ', latref,')')
	print(' -longrf : Longitude reference (for plot. current: ', longref,')')
	print(' -v : display information')
	print(' -vobj : display GPS coordinate in defined .txt path')
 	print(' -vobs : display square obstacles using GPS coordinate in defined .txt path')
	print(' ')
	print('WARNING: value of -head and -tw must be enter in degree')
	print(' ')


	# Local parameter
	t = 0
	dt = 0.01


	# intialisation sailboat
    	S1 = Sailboat(lat0,long0,theta0,psi_tw,atw,dt)


	# plot (0,0) initialization
	figure(1)
	plot(0,0,'*b')
	show(block=False)


	rospy.init_node('sailboat', anonymous=True)
	rate = rospy.Rate(1/dt)
	msg1 = msg()
	listener_input()


	while not rospy.is_shutdown():


		# listener_input()
	        usail =  msg1.y
        	urudder = msg1.x


		# Evaluate speed and position of the sailboat
                x0 = S1.x
                y0 = S1.y
        	S1.dynamic_sailboat(usail,urudder)

		# msg = 'wind information'
        	msg_Wind = Pose2D()
        	msg_Wind.x = S1.aaw
        	msg_Wind.theta = S1.psi_aw

		# msg = 'sail angle and rudder angle'
        	msg_sail = S1.deltas
        	msg_rudder = S1.deltar

		# msg  = 'GPS information'
		lat2, long2 = CartesienToGPS(S1.lat,S1.long,x0,y0,S1.x,S1.y)
		S1.lat = lat2
		S1.long = long2

        	msg_GPS =  GPSFix()
        	msg_GPS.latitude = S1.lat
        	msg_GPS.longitude = S1.long

		# msg = 'Imu informatiom'
	        x,y,z,w = EulerQuaternion(0.0,0.0,S1.theta)
        	msg_Imu = Imu()
        	msg_Imu.orientation.x = x
        	msg_Imu.orientation.y = y
        	msg_Imu.orientation.z = z
        	msg_Imu.orientation.w = w
		msg_Imu.linear_acceleration.x = S1.dv
		msg_Imu.linear_acceleration.y = S1.du


		# msg = 'velMsg'
		msg_Vel = Twist()
		msg_Vel.linear.x = S1.v
		msg_Vel.linear.y = S1.u

		# transmit message
        	talker_GPS(msg_GPS)
        	talker_IMU(msg_Imu)
        	talker_Wind(msg_Wind)
		talker_rudder_sail(msg_rudder,msg_sail)
		talker_velMsg(msg_Vel)

		if display == True:
                	print('command received sail/rudder: ',180.0/pi*usail, ' deg/ ',180.0/pi*urudder, ' deg')
			print('lat = ', S1.lat)
			print('long = ', S1.long)
			print('heading = ', S1.theta*180/pi,' deg')
	        	print('x = ', S1.x, ' m')
	        	print('y = ', S1.y, ' m')
			print('v = ', S1.v, ' m/s')
			print('u = ', S1.u, ' m/s')
	        	print('sail angle = ', S1.deltas*180/pi,' deg')
        		print('rudder angle = ', S1.deltar*180/pi,' deg')
			print(' ')

		# Plot sailboat
		dst = utilities.GPSDist(S1.lat,S1.long,latref,longref)
		theta00 = utilities.GPSBearing(latref,longref,S1.lat,S1.long)
		x00 = np.cos(theta00)*dst
		y00 = np.sin(theta00)*dst
		Lx = Lx + [x00]
		Ly = Ly + [y00]

		t = t+1
		if np.mod(t,100):

     	   		plot_sailboat(Lx,Ly,x00,y00,S1.theta,S1.deltas,S1.deltar,psi_tw,S1.aaw,display_obj,LObj,display_obs,LObs)
			pause(0.00001)

        	rate.sleep()
