#!/usr/bin/env python
import math
import numpy as np
import os
import rospkg
import sys

this = sys.modules[__name__]
this.config = {}

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
        ysqr = y * y
        t0 = +2.0 * (x * w + z * y)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.atan2(t3, t4)

        return X, Y, Z

def EulerToQuaternion(x, y, z):
        cy = math.cos(z * 0.5)
        sy = math.sin(z * 0.5)
        cr = math.cos(y * 0.5)
        sr = math.sin(y * 0.5)
        cp = math.cos(x * 0.5)
        sp = math.sin(x * 0.5)

        qw = cy * cr * cp + sy * sr * sp
        qx = cy * sr * cp - sy * cr * sp
        qy = cy * cr * sp + sy * sr * cp
        qz = sy * cr * cp - cy * sr * sp

        return qx,qy,qz,qw

def readGPSCoordinates(filepath):
	rospack = rospkg.RosPack()
	rospath = rospack.get_path('sailrobot')
	if filepath[0] is not '/':
		filepath = rospath + "/" + filepath
        if os.path.exists(filepath):
                with open(filepath, 'r') as file:
                        try:
                                res = []
                                problem = False
                                for line in file:
                                        coords = line.split(",")
                                        try:
                                                res.append([float(coords[0]),float(coords[1])])
                                        except:
                                                problem = True
                                if problem:
                                        print("Reading GPS : Wrong File Format for some lines")
                                return res
                        except:
                                print("Reading GPS : File not opened")
        else:
                print("Reading GPS : File does not exist")
        return []

def appendGPSCoordinates(filepath, res):
	rospack = rospkg.RosPack()
	rospath = rospack.get_path('sailrobot')
	if filepath[0] is not '/':
		filepath = rospath + "/" + filepath
        if os.path.exists(filepath):
                with open(filepath, 'r') as file:
                        try:
                                problem = False
                                for line in file:
                                        coords = line.split(",")
                                        try:
                                                res.append([float(coords[0]),float(coords[1])])
                                        except:
                                                problem = True
                                if problem:
                                        print("Reading GPS : Wrong File Format for some lines")
                                return res
                        except:
                                print("Reading GPS : File not opened")
        else:
                print("Reading GPS : File does not exist")
        return []

def ReadConfig(filepath):
    rospack = rospkg.RosPack()
    rospath = rospack.get_path('sailrobot')
    if filepath[0] is not '/':
        filepath = rospath + "/" + filepath
    if os.path.exists(filepath):
        with open(filepath, 'r') as file:
            try:
                res = {}
                problem = False
                for line in file:
                    keyval = line.split("=")
                    try:
                        res[keyval[0]]=keyval[1];
                    except:
                        problem = True
                if problem:
                    print("Reading Config : Wrong File Format for some lines")
                this.config = res
                return res
            except:
                print("Reading Config : File not opened")
    else:
        print("Reading Config : File does not exist")
    return []

def RelativeToTrueWind(v, heading, windDirection, windAcc):
    if(int(config["true_wind"])):
        angle = heading+windDirection
        angle = (angle + np.pi) % (2 * np.pi) - np.pi
        s = np.linalg.norm(v)
        windA = np.linalg.norm(windA)
        dx = s*math.cos(heading)-windA*math.cos(heading)
        dy = s*math.sin(heading)-windA*math.sin(heading)
        return math.atan2(dx,dy)
    else
        angle = heading+windDirection
        angle = (angle + np.pi) % (2 * np.pi) - np.pi
        return angle

def TackingStrategy(distanceToLine, lineBearing, windNorth, heading, corridor, psi, ksi){
    if(math.abs(distanceToLine) > corridor/2)
        q = 1 if distanceToLine >= 0 else -1

    if(math.cos(windNorth-heading)+math.cos(ksi) < 0 || (math.abs(distanceToLine) < corridor && (math.cos(windNorth-lineBearing)+math.cos(ksi) < 0)))
        heading = math.pi + windNorth - q*ksi;
    return heading,q

def StandardCommand(currentHeading, headaing, windNorth, max_sail, max_rudder):
    if(math.cos(currentHeading-heading)>=0):
        rudder=max_rudder*math.sin(currentHeading-heading)
    else
        rudder=max_rudder*math.copysign(1,sin(currentHeading-heading))

    sail = math.abs(max_sail*(math.cos(windNorth-heading)+1)/2.0)
    return rudder,sail
