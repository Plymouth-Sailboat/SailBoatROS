#!/usr/bin/env python
# license removed for brevity

import sys
from controller import Controller
from controller import MODE
import rospy
import utilities
import numpy as np
from math import pi
import math
from geometry_msgs.msg import Twist
import rospkg
from parameters import *
from Obstacle_avoidance import *

import Interval as py


# POUR COORDONNEES GPS


def CartesienToGPS(lat1,long1,x0,y0,x,y):

        Rearth = 6371000

        lat2 = lat1 + 180/pi*(x - x0)/Rearth
        long2 = long1 - 180/pi*(y - y0)/(Rearth*np.cos((lat1+lat2)/2))

        return lat2,long2



class Running(Controller):

        Xobj = 0
        Yobj = 0

        u = 0
        v = 0
        dv = 0
        q = 1

        deltarmax = deltarmax
        delta = delta
        darret0 = darret0
	dsecuObs = dsecuObs


        def __init__(self, name, nObj, LObj,looprate,display,rmax,LObs):
		Controller.__init__(self,name, looprate, MODE.RUDDER_SAIL)
                self.dt = looprate
                self.Xobj = LObj[0][0]
                self.Yobj = LObj[0][1]
		self.display = display
		self.nObj = 0
		self.MaxnObj = nObj
		self.x0 = LObj[1][0]
		self.y0 = LObj[1][1]
		self.rmax = rmax

		self.h = 0
		self.dsecuObs = dsecuObs
		self.deltarh = 0
		self.LObs = LObs
		self.List_dist = []



	def add_dynamic_obstacle(self):

                fileObs = rospack.get_path('sailrobot') + '/data/coord_Obstacle_dynamic.txt'
                LObs0 = utilities.readGPSCoordinates(fileObs)
		LObs1 = []

                if len(LObs0)<1:
                        LObs1 = []
                else:
                        i = 0
                        while i < len(LObs0):

				r_obs = 0.5; # obstacle of radium 1m and center the gps coord. ((improved))
				[lat1,long1] = CartesienToGPS(LObs0[i][0],LObs0[i][1],0,0,r_obs*math.sqrt(2)/2,r_obs*math.sqrt(2)/2)
				[lat2,long2] = CartesienToGPS(LObs0[i][0],LObs0[i][1],0,0,-r_obs*math.sqrt(2)/2,-r_obs*math.sqrt(2)/2)

                                latmin, latmax = np.min([lat1, lat2]), np.max([lat1, lat2])
                                longmin, longmax = np.min([long1, long2]), np.max([long1, long2])
                                Obs = py.IntervalVector([py.Interval(latmin,latmax),py.Interval(longmin,longmax)])

                                i = i+1
                                LObs1 = LObs1 + [Obs]

		LObsTot = self.LObs + LObs1


		return LObsTot




        def update_rudder(self,thetab,theta,phi):

		psi = 0
                deltar = 0

                if np.cos(theta - phi) > np.cos(pi/8) :
                	psi = phi
                else:
                        psi = theta

                # calcul de deltar
                if np.cos(psi - thetab) >= 0 :
                        deltar = self.deltarmax*(np.sin(psi - thetab))
                else:
                        deltar = self.deltarmax*np.sign(np.sin(psi - thetab))
                return deltar


        def desired_orientation(self,thetab0,alpha,psi_tw,x,y,phi):   # first desired orientation
                thetab = thetab0

                # test if desired orientation is upwind: tack strategy
                if np.cos(psi_tw - thetab) + np.cos(self.delta) < 0 :

	               	if self.display == True:
        	               	print('Tack strategy')


	        #        dst =  utilities.GPSDist(x, y, self.Xobj, self.Yobj)
        	#        alpha1 = utilities.GPSBearing(self.Xobj, self.Yobj, x, y)
		#	alpha0 = utilities.GPSBearing(self.Xobj, self.Yobj, self.x0, self.y0)
		#	alphai = alpha1 - alpha0

		#	e = dst*np.abs(np.sin(alphai))
		#	if e > self.rmax:
		#		self.q = np.sign(np.sin(alphai))

                #       thetab = psi_tw + pi + self.q*self.delta


			###############
			if np.cos(thetab0-(psi_tw+pi+self.delta))>np.cos(thetab0-(psi_tw+pi-self.delta)):
				thetab = psi_tw+pi+self.delta
			else:
				thetab = psi_tw+pi-self.delta
			#############

		# obstacle avoidance
		param2 = [dsecu,dsecu2,p8*2]

		thetaTest = thetab
		List_Obs = self.add_dynamic_obstacle()
		#List_Obs = self.LObs
		thetab, self.List_dist = Obstacle_avoidance(x,y,thetab,phi,psi_tw,self.delta,dsecuObs,param2,List_Obs)

                if (self.display == True)&(np.cos(thetaTest-thetab)<cos(pi/10)):
                        print('OBSTACLE AVOIDANCE')
			print(thetaTest*180/pi,thetab*180/pi)
		return thetab


        def sail_opening(self,theta,psi_tw,psi_aw,thetab):


                # Optimal sail angle
                deltasopt = np.abs(pi/2*(np.cos(psi_aw) + 1)/2)
                deltasb = np.min([deltasopt,np.abs(pi-np.abs(psi_aw))-pi/36])
                deltasb = - np.sign(psi_aw)*np.min([np.abs(deltasb),pi/2])

                return deltasb


        def control(self):

                # load information of sailboat
                x,y = self.gpsMsg.latitude,  self.gpsMsg.longitude
                xi,yi,zi,wi = self.imuMsg.orientation.x, self.imuMsg.orientation.y, self.imuMsg.orientation.z, self.imuMsg.orientation.w
                theta = utilities.QuaternionToEuler(xi, yi, zi, wi)[2]

                self.dv = self.imuMsg.linear_acceleration.x
                self.v,self.u = self.velMsg.linear.x, self.velMsg.linear.y

                aaw = np.max([self.windMsg.x,1])
		psi_aw = self.windMsg.theta

                # evaluation of phi
                dx = self.v*np.cos(theta) - self.u*np.sin(theta)
                dy = self.v*np.sin(theta) + self.u*np.cos(theta)
                #phi = math.atan2(dy,dx) # phi = np.angle(dx + 1j*dy)
		phi = theta

                # calcul de psi_tw ((en attendant une autre version du code))
                u0 = self.v*np.sin(phi) + aaw*np.sin(theta + psi_aw)
                v0 = self.v*np.cos(phi) + aaw*np.cos(theta + psi_aw)

                psi_tw = math.atan2(u0,v0)

                # evaluate distance and angle between taregt and ship
                dst =  utilities.GPSDist(x, y, self.Xobj, self.Yobj)
                alpha = utilities.GPSBearing(self.Xobj, self.Yobj, x, y)
                thetab0 = utilities.GPSBearing(x, y, self.Xobj, self.Yobj)

		if dst < darret0:
			if self.nObj == self.MaxnObj-1:
				self.nObj = 0
                                self.x0 = self.Xobj
                                self.y0 = self.Yobj
				self.Xobj = LObj[0][0]
				self.Yobj = LObj[0][1]

			else :
				self.nObj = self.nObj+1
                                self.x0 = self.Xobj
                                self.y0 = self.Yobj
				self.Xobj = LObj[self.nObj][0]
				self.Yobj = LObj[self.nObj][1]


                # evaluate the desire orientation
                thetab = self.desired_orientation(thetab0,alpha,psi_tw,x,y,phi)

                # evaluate sail opening
                deltasb = self.sail_opening(theta,psi_tw,psi_aw,thetab)

                # evaluate rudder angle
		deltarb = Fct_update_rudder(self.List_dist,thetab,theta,phi,self)

                # send message to update control of sail and rudder
                command = Twist()
                command.angular.x = deltarb
                command.angular.y = deltasb

                if self.display == True:
                        print('Current Xobj = ', self.Xobj,'/Yobj = ',self.Yobj)
			print('Current position = ', x,', ',y)
                        print('dst obj = ', dst)
			print('theta obj = ', thetab*180/pi)
                        print('headind received = ',theta*180/pi)
			print('thetab0 = ', thetab0*180/pi)
                        print('alpha = ', alpha*180/pi)
                        print('desired deltas = ', deltasb*180/pi)
                        print('desired deltar = ', deltarb*180/pi)
                        print('evaluate psi_tw = ', psi_tw*180/pi)
                        print(' ')
		self.publishMSG('Pdist '+ (str)(dst) + '\n thetaobj ' + (str)(thetab*180/pi) + '\n Xobj ' + (str)(self.Xobj) + ' Yobj ' + (str)(self.Yobj))
                return command



if __name__ == '__main__':


        try:
		test_ls = 0
                rospack = rospkg.RosPack()
                fileObs = rospack.get_path('sailrobot') + '/data/coord_Obstacle.txt'
                LObs0 = utilities.readGPSCoordinates(fileObs)
		LObs = []
		test_GPS_Obs = False

                fileGPS = rospack.get_path('sailrobot') + '/data/gps_running.txt'
                LObj = utilities.readGPSCoordinates(fileGPS)
                nObj = len(LObj)
                if nObj <2:
                        LObj = array([LObj[0],LObj[0]])
                        nObj = nObj+1


		if len(LObs0)<2:
			LObs = []
		else:
			i = 0
			while i < len(LObs0)-1:
				#Obs = py.IntervalVector([py.Interval(LObs0[i][0],LObs0[i][1]),py.Interval(LObs0[i+1][0],LObs0[i+1][1])])
				latmin, latmax = np.min([LObs0[i][0], LObs0[i+1][0]]), np.max([LObs0[i][0], LObs0[i+1][0]])
				longmin, longmax = np.min([LObs0[i][1], LObs0[i+1][1]]), np.max([LObs0[i][1], LObs0[i+1][1]])

				Obs = py.IntervalVector([py.Interval(latmin,latmax),py.Interval(longmin,longmax)])

				i = i+2
				LObs = LObs + [Obs]



                display = False
                rate = 10
		rmax = 50
		test_GPS_file = False

                for i in range(0,len(sys.argv)):
 			if sys.argv[i] == '-n':
				if nObj < 0: #sys.argv[i+1]:
					print('Not enough objective implement! Take n= ',nObj)
				else:
					nObj = float(sys.argv[i+1])

			if sys.argv[i] == '-gpsfile':
				test_GPS_file = True
				if sys.argv[i+1] == '0':
					LObj = utilities.readGPSCoordinates(fileGPS)
					nObj = len(LObj)

				elif sys.argv[i+1] == '/':
					LObj = utilities.readGPSCoordinates(sys.argv[i+1])
					nObj = len(LObj)

				else:
                                        fileGPS = rospack.get_path('sailrobot')+ '/data/' + sys.argv[i+1]
                                        LObj = utilities.readGPSCoordinates(fileGPS)
                                        nObJ = len(LObj)


                                if nObj <2:
                                        LObj = array([LObj[0],LObj[0]])
                                        nObj = nObj+1



			if sys.argv[i] == '-gpsobstacle':

				test_GPS_Obs = True
                                if sys.argv[i+1] == '/':
                                        LObs0 = utilities.readGPSCoordinates(sys.argv[i+1])

                                else:
                                        fileGPS = rospack.get_path('sailrobot')+ '/data/' + sys.argv[i+1]
                                        LObs0 = utilities.readGPSCoordinates(fileGPS)


                		LObs = []
                		if len(LObs0)<2:
                        		LObs = []
                		else:
                        		j = 0
                        		while j < len(LObs0)-1:


                                		latmin, latmax = np.min([LObs0[j][0], LObs0[j+1][0]]), np.max([LObs0[j][0], LObs0[j+1][0]])
                                		longmin, longmax = np.min([LObs0[j][1], LObs0[j+1][1]]), np.max([LObs0[j][1], LObs0[j+1][1]])
                                		Obs = py.IntervalVector([py.Interval(latmin,latmax),py.Interval(longmin,longmax)])
                                		j = j+2
                                		LObs = LObs + [Obs]


			if sys.argv[i] == '-rate':
                                rate = float(sys.argv[i+1])

                        if sys.argv[i] == '-v':
                                display = True

			if sys.argv[i] == '-rm':
				rmax = float(sys.argv[i+1])

                        if sys.argv[i] == '-ls':
                                test_ls = 1


		print(' -n : number of objective')
                print(' -rate : loop rate' )
                print(' -v : display information')
		print(' -gpsfile : filepath of GPS coordinate')
		print(' -gpsobstacle : filepath of obstacle GPS coordinate')
		print(' -rm : cutoff distance')
                print('-ls : display list of function' )
                print(' ')


                if (test_ls == 1):
                        print(' ')
                else:


			if test_GPS_file == False:
				print('Default GPS coordinate of file gps_running.txt used. Enter filepath of an other file with command -gpsfile if desire.')
				print(' ')

			if test_GPS_Obs == False:
        	                print('Default GPS coordinate Obstacle of file coord_Obstacle.txt used. Enter filepath of an other file with command -gpsobstacle if desire.')
                	        print(' ')


			target =  Running('running', nObj,LObj, rate,display,rmax,LObs)


        	        while not rospy.is_shutdown():
				target.loop()


        except rospy.ROSInterruptException:

                pass

