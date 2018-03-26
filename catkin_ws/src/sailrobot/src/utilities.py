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
