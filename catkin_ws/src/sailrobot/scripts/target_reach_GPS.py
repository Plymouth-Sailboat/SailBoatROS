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


# POUR COORDONNEES GPS


class Target_Reach(Controller):

        Xobj = 0
        Yobj = 0
        u = 0
        v = 0
        dv = 0
        q = 1

#        p8 = 0.1
#        deltarmax = pi/4
#        delta = pi/4
#        dsecu = 65
#        darret0 = 5
#        dsecu2 = 15
#        kp = 1
#        kv = 8
#        ka = 5

        p8 = p8
        deltarmax = deltarmax
        delta = delta
        dsecu = dsecu
        darret0 = darret0
        dsecu2 = dsecu2
        kp = kp
        kv = kv
        ka = ka



        def __init__(self, name, X, Y, looprate,display):
		Controller.__init__(self,name, looprate, MODE.RUDDER_SAIL)
                self.dt = looprate
                self.Xobj = X
                self.Yobj = Y
		self.display = display
		self.darret = np.max([self.darret0, self.p8/np.sin(pi/4)])


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

        def desired_orientation(self,thetab0,dst,alpha,psi_tw):   # first desired orientation 
                thetab = thetab0

                # Bypass strategy
                if (dst< self.dsecu)&(dst>self.dsecu2)&(np.cos(thetab - psi_tw)>0):
                        if np.cos(alpha - (psi_tw + pi/2)) > np.cos(alpha - (psi_tw - pi/2)):
                                thetab = thetab + pi/2
                        else:
                                thetab = thetab - pi/2

                # test if desired orientation is upwind: tack strategy
                if np.cos(psi_tw - thetab) + np.cos(self.delta) < 0 :
                        thetab = psi_tw + pi + self.q*self.delta
                else:   # else, stock sailboat direction

                        if (np.cos(thetab - (psi_tw + pi - self.delta))> np.cos(thetab - (psi_tw  + pi + self.delta))):
                        	self.q = - 1
                        else:
                                self.q = 1

                # stop strategy : put sailboat upwind
                if (dst < self.darret):
                        if (self.v > 0):
                                thetab = psi_tw + pi
                        else:
                                thetab = psi_tw + pi - self.delta*np.sign(self.u)

                return thetab

        def sail_opening(self,theta,dst,psi_tw,psi_aw,dvd,thetab):
                deltas = self.sailAngle


                # stop strategy : put sailboat upwind
                if (dst < self.darret):
                        if (self.v > 0):
                                deltasb = -np.sign(psi_aw)*np.abs(pi-np.abs(psi_aw))
                        	if np.abs(deltasb)>pi/2:
					deltasb = sign(deltasb)*pi/2

			else:
                                deltasopt = np.abs(pi/2*(np.cos(psi_tw - thetab) + 1)/2)
                                deltasM = np.min([pi/2,np.abs(pi - np.abs(psi_aw))])
                                eps = pi/36
                                deltaslim = np.max([0, deltasM-eps])
                                deltasb = - np.sign(psi_aw)*np.min([deltasopt,deltaslim])

                else:  # if ship far to the target area
                        ddeltas = -(dvd - self.dv)*self.ka
                        deltasb = np.abs(deltas) + self.dt*ddeltas


                	if deltasb < 0:
                        	deltasb = 0

                	if ddeltas < 0:  # desire to speed up
                        	deltasopt = np.abs(pi/2*(np.cos(psi_tw - thetab) + 1)/2)
                        	deltasb = np.max([np.abs(deltasb), np.min([deltasopt,np.abs(pi-np.abs(psi_aw))-pi/36])]) 
                        	deltasb = - np.sign(psi_aw)*np.min([np.abs(deltasb),pi/2])
                	else:   # desire to slow down
                        	borne2 = np.max([np.abs(pi-np.abs(psi_aw)),0.0])
                        	deltasb = - np.sign(psi_aw)*np.min([np.abs(deltasb),borne2,pi/2])

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

                # evaluate desire acceleration
                v0 = np.max([0, self.u*np.tan(theta - alpha)])
                dvd = -self.kv*(self.v- v0) + self.kp*dst

                # evaluate the desire orientation
                thetab = self.desired_orientation(thetab0,dst,alpha,psi_tw)

                # evaluate sail opening
                deltasb = self.sail_opening(theta,dst,psi_tw,psi_aw,dvd,thetab)

                # evaluate rudder angle
                deltarb = self.update_rudder(thetab,theta,phi)

                # send message to update control of sail and rudder
                command = Twist()
                command.angular.x = deltarb
                command.angular.y = deltasb

		if self.display == True:
                	print('Xobj = ', self.Xobj,'/Yobj = ',self.Yobj)
			print('dst obj = ', dst)
			print('theta obj = ',thetab*180/pi)
        	        print('alpha = ', alpha*180/pi)
			print('desired deltas = ', deltasb*180/pi)
			print('desired deltar = ', deltarb*180/pi)
        	        print('evaluate psi_tw = ', psi_tw*180/pi)
			print(' ')

                return command


if __name__ == '__main__':


	try:
		lat0 = 0
		long0 = 0
		display = False
		rate = 10
		testlat = 0
		testlong = 0

        	for i in range(0,len(sys.argv)):
                	if sys.argv[i] == '-lat':
                        	lat0 = float(sys.argv[i+1])
				testlat = 1
	                if sys.argv[i] == '-long':
        	                long0 = float(sys.argv[i+1])
				testlong = 1

	                if sys.argv[i] == '-rate':
        	                rate = float(sys.argv[i+1])

			if sys.argv[i] == '-v':
				display = True

			if sys.argv[i] == '-gpsfile':
				LObj = utilities.readGPSCoordinates(sys.argv[i+1])
				lat0 = LObj[0][0]
				long0 = LObj[0][1]
				testlat = 0
				testlong = 0
		
		print('-lat : latitude objective')
		print('-long : longitude objective')
		print('-rate : loop rate' )
		print('-v : display information')
		print(' -gpsfile : filepath of GPS coordinate')
		print(' ')


		if (testlat == 0)|(testlong == 0):
			print("Add Latitude obj and Longitude obj")

		else:
		        target =  Target_Reach('target', lat0, long0,rate,display)
                        while not rospy.is_shutdown():
                                target.loop()


	except rospy.ROSInterruptException:

		pass
