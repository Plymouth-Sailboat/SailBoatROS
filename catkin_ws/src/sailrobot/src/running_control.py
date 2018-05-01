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


        def __init__(self, name, nObj,looprate,display):
		Controller.__init__(self,name, looprate, MODE.RUDDER_SAIL)
                self.dt = looprate
                self.Xobj = LObj[0][0]
                self.Yobj = LObj[0][1]
		self.display = display
		self.nObj = 0
		self.MaxnObj = nObj



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


        def desired_orientation(self,thetab0,alpha,psi_tw):   # first desired orientation
                thetab = thetab0


                # test if desired orientation is upwind: tack strategy
                if np.cos(psi_tw - thetab) + np.cos(self.delta) < 0 :
                        thetab = psi_tw + pi + self.q*self.delta

                else:   # else, stock sailboat direction

                        if (np.cos(thetab - (psi_tw + pi - self.delta))> np.cos(thetab - (psi_tw  + pi + self.delta))):
                        	self.q = - 1
                        else:
                                self.q = 1

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
				self.Xobj = LObj[0][0]
				self.Yobj = LObj[0][1]

			else :
				self.nObj = self.nObj+1
				self.Xobj = LObj[self.nObj][0]
				self.Yobj = LObj[self.nObj][1]


                # evaluate the desire orientation
                thetab = self.desired_orientation(thetab0,alpha,psi_tw)

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
                        print('alpha = ', alpha*180/pi)
                        print('desired deltas = ', deltasb*180/pi)
                        print('desired deltar = ', deltarb*180/pi)
                        print('evaluate psi_tw = ', psi_tw*180/pi)
                        print(' ')

                return command



if __name__ == '__main__':


        try:

                display = False
                rate = 10
		nObj = len(LObj)

                for i in range(0,len(sys.argv)):
 			if sys.argv[i] == '-n':
				if nObj >= sys.argv[i+1]:
					print('Not enough objective implement! Take n= ',nObj)
				else:
					nObj = float(sys.argv[i+1])


                        if sys.argv[i] == '-rate':
                                rate = float(sys.argv[i+1])

                        if sys.argv[i] == '-v':
                                display = True

		print(' -n : number of objective')
                print(' -rate : loop rate' )
                print(' -v : display information')
                print(' ')


                target =  Running('running', nObj,rate,display)
                while not rospy.is_shutdown():
			target.loop()


        except rospy.ROSInterruptException:

                pass

