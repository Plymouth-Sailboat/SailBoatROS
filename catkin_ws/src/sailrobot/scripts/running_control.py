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


        def __init__(self, name, nObj, LObj, looprate,display,rmax):
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


        def update_rudder(self,thetab,theta,phi):

		psi = 0
                deltar = 0

                if np.cos(theta - phi) > np.cos(pi/8) :
                	psi = phi
                else:
                        psi = theta

		#psi = theta;

                # calcul de deltar
                if np.cos(psi - thetab) >= 0 :
                        deltar = self.deltarmax*(np.sin(psi - thetab))
                else:
                        deltar = self.deltarmax*np.sign(np.sin(psi - thetab))
                return deltar


        def desired_orientation(self,thetab0,alpha,psi_tw,x,y):   # first desired orientation
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
                thetab = self.desired_orientation(thetab0,alpha,psi_tw,x,y)

                # evaluate sail opening
                deltasb = self.sail_opening(theta,psi_tw,psi_aw,thetab)

                # evaluate rudder angle
                deltarb = self.update_rudder(thetab,theta,phi)

                # send message to update control of sail and rudder
                command = Twist()
                command.angular.x = deltarb
                command.angular.y = deltasb

                if self.display == True:
                        print('Current Xobj = ', self.Xobj,'/Yobj = ',self.Yobj)
                        print('dst obj = ', dst)
			print('theta obj = ', thetab*180/pi)
                        print('headind received = ',theta*180/pi)
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
                display = False
                rate = 10
		rmax = 50
		rospack = rospkg.RosPack()
		fileGPS = rospack.get_path('sailrobot') + '/data/gps_running.txt'
                LObj = utilities.readGPSCoordinates(fileGPS)
                nObj = len(LObj)
                if nObj <2:
			LObj = array([LObj[0],LObj[0]])
                        nObj = nObj+1
#		LObj = []
#		nObj = 0
		test_GPS_file = False

                for i in range(0,len(sys.argv)):

			if sys.argv[i] == '-gpsfile':
				test_GPS_file = True
				path = sys.argv[i+1]
				if path == '0':
					LObj = utilities.readGPSCoordinates(fileGPS)
					nObj = len(LObj)

				elif path[0]=='/':

					LObj = utilities.readGPSCoordinates(sys.argv[i+1])
					nObj = len(LObj)

				else :
                                        fileGPS = rospack.get_path('sailrobot')+ '/data/' + sys.argv[i+1]
                                        LObj = utilities.readGPSCoordinates(fileGPS)
                                        nObj = len(LObj)


				if nObj <2:
					LObj = array([LObj[0],LObj[0]])
					nObj = nObj+1


                        if sys.argv[i] == '-rate':
                                rate = float(sys.argv[i+1])

                        if sys.argv[i] == '-v':
                                display = True

			if sys.argv[i] == '-rm':
				rmax = float(sys.argv[i+1])

                        if sys.argv[i] == '-ls':
                                test_ls = 1


                print(' -rate : loop rate' )
                print(' -v : display information')
		print(' -gpsfile : filepath of GPS coordinate')
		print(' -rm : cutoff distance')
                print('-ls : display list of function' )
                print(' ')

                if (test_ls == 1):
                        print(' ')
                else:

			if test_GPS_file == False:
				print('Default GPS coordinate of file gps_running.txt used. Enter filepath of an other file with command -gpsfile if desire.')
				print(' ')

			target =  Running('running', nObj, LObj,rate,display,rmax)


        	        while not rospy.is_shutdown():
				target.loop()


        except rospy.ROSInterruptException:

                pass

