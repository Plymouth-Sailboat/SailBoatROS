#!/usr/bin/env python
import math

def GPSDist(lat1, lon1, lat2, lon2):
	R = 6371000
        ksi1 = lat1*math.pi/180.0
        ksi2 = lat2*math.pi/180.0
        dksi = (lat2-lat1)*math.pi/180.0
        dlambda = (lon2-lon1)*math.pi/180.0

        a = math.sin(dksi/2.0)*math.sin(dksi/2.0) + math.cos(ksi1)*math.cos(ksi2)*math.sin(dlambda/2.0)*math.sin(dlambda/2.0)
        c = 2*math.atan2(sqrt(a), sqrt(1-a))
        return R*c

def GPSDistFast(lat1, lon1, lat2, lon2):
	ksi1 = lat1*math.pi/180.0
        ksi2 = lat2*math.pi/180.0
        lam1 = lon1*math.pi/180.0
        lam2 = lon2*math.pi/180.0

        x = (lam2-lam1)*math.cos((ksi1+ksi2)/2.0)
        y = ksi2-ksi1
        return math.sqrt(x*x + y*y) * 6371000

def GPSBearing(lat1, lon1, lat2, lon2):
	ksi1 = lat1*math.pi/180.0
        ksi2 = lat2*math.pi/180.0
        lam1 = lon1*math.pi/180.0
        lam2 = lon2*math.pi/180.0

        y = math.sin(lam2-lam1)*math.cos(ksi2)
        x = math.cos(ksi1)*math.sin(ksi2)-math.sin(ksi1)*math.cos(ksi2)*math.cos(lam2-lam1)
        return math.atan2(y,x)

def QuaternionToEuler(w, x, y, z):
	ysqr = y * y
	
	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = math.degrees(math.atan2(t0, t1))
	
	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.degrees(math.asin(t2))
	
	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = math.degrees(math.atan2(t3, t4))
	
	return X, Y, Z
