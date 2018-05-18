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

from parameters import *
from Obstacle_avoidance import *

import Interval as py


# POUR COORDONNEES GPS


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

	                dst =  utilities.GPSDist(x, y, self.Xobj, self.Yobj)
        	        alpha1 = utilities.GPSBearing(self.Xobj, self.Yobj, x, y)
			alpha0 = utilities.GPSBearing(self.Xobj, self.Yobj, self.x0, self.y0)
			alphai = alpha1 - alpha0

			e = dst*np.abs(np.sin(alphai))
			if e > self.rmax:
				self.q = np.sign(np.sin(alphai))

                        thetab = psi_tw + pi + self.q*self.delta

		# obstacle avoidance
		param2 = [dsecu,dsecu2,p8*2]
		thetab, self.List_dist = Obstacle_avoidance(x,y,thetab,phi,psi_tw,delta,dsecuObs,param2,self.LObs)

		return thetab


        def sail_opening(self,theta,psi_tw,psi_aw,thetab):


                # Optimal sail angle
                deltasopt = np.abs(pi/2*(np.cos(psi_tw - thetab) + 1)/2)
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

                aaw = self.windMsg.x
		psi_aw = self.windMsg.theta

                # evaluation of phi
                dx = self.v*np.cos(theta) - self.u*np.sin(theta)
                dy = self.v*np.sin(theta) + self.u*np.cos(theta)
                phi = math.atan2(dy,dx) # phi = np.angle(dx + 1j*dy)


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
#                deltarb = self.update_rudder(thetab,theta,phi)
		deltarb = Fct_update_rudder(self.List_dist,thetab,theta,phi,self)

                # send message to update control of sail and rudder
                command = Twist()
                command.angular.x = deltarb
                command.angular.y = deltasb

                if self.display == True:
                        print('Current Xobj = ', self.Xobj,'/Yobj = ',self.Yobj)
                        print('dst obj = ', dst)
			print('theta obj = ', thetab*180/pi)
                        print('alpha = ', alpha*180/pi)
                        print('desired deltas = ', deltasb*180/pi)
                        print('desired deltar = ', deltarb*180/pi)
                        print('evaluate psi_tw = ', psi_tw*180/pi)
			print('headind received = ',theta*180/pi)
                        print(' ')

                return command



if __name__ == '__main__':


        try:

                fileObs = '/home/sailboat/git/SailBoatROS/catkin_ws/src/sailrobot/scripts/coord_Obstacle.txt'
                LObs0 = utilities.readGPSCoordinates(fileObs)
		LObs = []
		if len(LObs0)< 2:
			LObs = []
		else:
			i = 0
			while i < len(LObs0)-1:
				Obs = py.IntervalVector([py.Interval(LObs0[i][0],LObs0[i][1]),py.Interval(LObs0[i+1][0],LObs0[i+1][1])])
				i = i+2
				LObs = LObs + [Obs]



                display = False
                rate = 10
		rmax = 50
		fileGPS = '/home/sailboat/git/SailBoatROS/catkin_ws/src/sailrobot/scripts/coord_GPS.txt'
		LObj = [] 
		nObj = 0
		test_GPS_file = False

                for i in range(0,len(sys.argv)):
 			if sys.argv[i] == '-n':
				if nObj >= sys.argv[i+1]:
					print('Not enough objective implement! Take n= ',nObj)
				else:
					nObj = float(sys.argv[i+1])

			if sys.argv[i] == '-gpsfile':
				if sys.argv[i+1] == '0':
					LObj = utilities.readGPSCoordinates(fileGPS)
					nObj = len(LObj)
				else:	
					LObj = utilities.readGPSCoordinates(sys.argv[i+1])
					nObj = len(LObj)

                        if sys.argv[i] == '-rate':
                                rate = float(sys.argv[i+1])

                        if sys.argv[i] == '-v':
                                display = True

			if sys.argv[i] == '-rm':
				rmax = float(sys.argv[i+1])


		print(' -n : number of objective')
                print(' -rate : loop rate' )
                print(' -v : display information')
		print(' -gpsfile : filepath of GPS coordinate')
		print(' -rm : cutoff distance')
                print(' ')


		if test_GPS_file == False:
			print('Default GPS coordinate of file coord_GPS.txt used. Enter filepath of an other file with command -gpsfile if desire.')
			print(' ')

		target =  Running('running', nObj,LObj, rate,display,rmax,LObs)


                while not rospy.is_shutdown():
			target.loop()


        except rospy.ROSInterruptException:

                pass

