#!/usr/bin/env python
import math
import numpy as np
import os
import rospkg

def GPSDist(lat1, lon1, lat2, lon2):
        R = 6371000
        degtorad = math.pi/180.0
        ksi1 = lat1*degtorad
        ksi2 = lat2*degtorad
        dksi = (lat2-lat1)*degtorad
        dlambda = (lon2-lon1)*degtorad

        a = math.sin(dksi/2.0)*math.sin(dksi/2.0) + math.cos(ksi1)*math.cos(ksi2)*math.sin(dlambda/2.0)*math.sin(dlambda/2.0)
        c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R*c

def GPSDistFast(lat1, lon1, lat2, lon2):
        degtorad = math.pi/180.0
        ksi1 = lat1*degtorad
        ksi2 = lat2*degtorad
        lam1 = lon1*degtorad
        lam2 = lon2*degtorad

        x = (lam2-lam1)*math.cos((ksi1+ksi2)/2.0)
        y = ksi2-ksi1
        return math.sqrt(x*x + y*y) * 6371000

def GPSBearing(lat1, lon1, lat2, lon2):
        degtorad = math.pi/180.0
        ksi1 = lat1*degtorad
        ksi2 = lat2*degtorad
        lam1 = lon1*degtorad
        lam2 = lon2*degtorad

        y = math.sin(lam2-lam1)*math.cos(ksi2)
        x = math.cos(ksi1)*math.sin(ksi2)-math.sin(ksi1)*math.cos(ksi2)*math.cos(lam2-lam1)
	bearing = -math.atan2(y,x)
        return bearing

def QuaternionToEuler(x,y,z,w):
        zsqr = z * z
	
        t0 = +2.0 * (x * y + z * w)
        t1 = +1.0 - 2.0 * (y * y + zsqr)
        X = math.atan2(t0, t1)
	
        t2 = +2.0 * (x * z - w * y)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)
	
        t3 = +2.0 * (x * w + y * z)
        t4 = +1.0 - 2.0 * (zsqr + w * w)
        Z = math.atan2(t3, t4)
	
        return X, Y, Z

def EulerToQuaternion(x, y, z):
        cy = math.cos(z * 0.5)
        sy = math.sin(z * 0.5)
        cr = math.cos(y * 0.5)
        sr = math.sin(y * 0.5)
        cp = math.cos(x * 0.5)
        sp = math.sin(x * 0.5)

        qx = cy * cr * cp + sy * sr * sp
        qy = cy * sr * cp - sy * cr * sp
        qz = cy * cr * sp + sy * sr * cp
        qw = sy * cr * cp - cy * sr * sp
	
        return qx,qy,wz,qw

def readGPSCoordinates(filepath):
	rospack = rospkg.RosPack()
	rospath = rospack.get_path('sailrobot')
	if filepath[0] is not '/':
		filepath = rospath + "/" + filepath
        if os.path.exists(filepath):
                with open(filepath, 'r') as file:
                        try:
                                res = []
                                noproblem = True
                                for line in file:
                                        coords = line.split(",")
                                        try:
                                                res.append([float(coords[0]),float(coords[1])])
                                        except:
                                                noproblem = False
                                if not noproblem:
                                        print("Reading GPS : Wrong File Format for some lines")
                                return res
                        except:
                                print("Reading GPS : File not opened")
        else:
                print("Reading GPS : File does not exist")
        return []
