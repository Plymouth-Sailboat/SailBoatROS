# -*- coding: utf-8 -*-
"""
Created on Tue Nov 14 14:40:53 2017

@author: christophe
"""

import math
import numpy as np
nan = float('nan')
oo = float( 'inf' )
from numpy import pi

class Interval:

    def __init__(x,a,b):
        if a>b:
            x.lb, x.ub = nan, nan
        else:
            x.lb, x.ub = a, b
            # x.lb : x "lower-bound", x.ub : x "upper bound"

    def __getitem__(x,a):
        if a == 0:
            return x.lb
        elif a == 1:
            return x.ub

    def __setitem__(x,c,d) :
        if c == 0:
            a = d
            b = x.ub

        elif c == 1:
            a = x.lb
            b = d

        if a>b:
            x.lb, x.ub = nan, nan
        else:
            x.lb, x.ub = a, b


    def is_empty(x):
        return math.isnan(x.lb)
    
    def width(x):
        return x.ub-x.lb
    
    def left(x):
        return Interval(x.lb,0.5*(x.lb+x.ub))
    
    def right(x):
        return Interval(0.5*(x.lb+x.ub),x.ub)


    def __add__(x,y):   # overlading x+y
        return Interval(x.lb+y.lb,x.ub+y.ub)

    # display 
    def __repr__(x):
        return ("[%f,%f]"%(x.lb,x.ub))

    #####################


    def __sub__(x,y):   # overlading x-y
        return Interval(x.lb-y.ub,x.ub-y.lb)

    def __mul__(x,y):   # overlading x*y
        L= [x.lb*y.lb, x.ub*y.lb, x.lb*y.ub, x.ub*y.ub]
        return Interval(np.min(L),np.max(L))


    def __rmul__(x,y):
        return x.__mul__(Interval(y,y))

    def __radd__(x,y):
        return x.__add__(Interval(y,y))

    def __rsub__(x,y):
        return (Interval(y,y)).__sub__(x)

    def __contains__(x,a):
        return (x.lb<=a<=x.ub)

    def __truediv__(x,y):
        if 0 in y:
            return Interval(-oo,oo)
        else:
            return x.__mul__(Interval(1./y.ub,1./y.lb))

    # Intersection between x and y
    def __and__(x,y):
        if (x.is_empty()) or y.is_empty():
            return Interval(1,0)
        else:
            return Interval(np.max([x.lb,y.lb]),np.min([x.ub,y.ub]))
    # union entre x and y
    def __or__(x,y):
        if x.is_empty() and y.is_empty():
            return Interval(1,0)
        elif (x.is_empty()):
            return y
        elif y.is_empty():
            return x
        else:
            return Interval(np.min([x.lb,y.lb]),np.max([x.ub,y.ub]))


    def is_subset(x,y):
        if x.is_empty():
            return True
        else:
            return (x.lb in y) and (x.ub in y)


    def is_disjoint(x,y):
        return (x&y).is_empty()




def sqr(x):
    L = [x.lb**2,x.ub**2]
    if 0 in x:
        return Interval(0,np.max(L))
    else:
        return Interval(np.min(L),np.max(L))

def sqrt(x):
    x = x&Interval(0,oo)
    return Interval(math.sqrt(x.lb),math.sqrt(x.ub))


def mini(x,y):
    return  Interval(np.min(x.lb,y.lb),np.min(x.ub,y.ub))

def maxi(x,y):
    return  Interval(np.max(x.lb,y.lb),np.max(x.ub,y.ub))


def exp(x):
    return Interval(math.exp(x.lb),math.exp(x.ub))

def log(x):
    if x.ub <= 0:
        return Interval(nan,nan)
    elif 0 in x:
        return Interval(-oo,math.log(x.ub))
    else:
        return Interval(math.log(x.lb),math.log(x.ub))


def cos(x):

    if x.ub > pi:
        x = -2*pi + x
    if x.lb < -pi:
        x = 2*pi + x

    if (x.ub > pi)&(x.lb < -pi):
        return Interval(-1,1)
    elif (x.ub > pi)|(x.lb < -pi):
        Lb = -1
        Ub = np.max([np.cos(x.lb),np.cos(x.ub)])
        return Interval(Lb,Ub)

    if 0 in x:
        Ub = 1
        Lb = np.cos(np.max([np.abs(x.ub),np.abs(x.lb)]))
    elif x.is_subset(Interval(0,pi)):
        Lb = np.cos(x.ub)
        Ub = np.cos(x.lb)
    elif x.is_subset(Interval(-pi,0)):
        Lb = np.cos(x.lb)
        Ub = np.cos(x.ub)

    return Interval(Lb,Ub)



#def sin(x):
#
#    if x.ub > pi:
#        x = -2*pi + x
#    if x.lb < -pi:
#        x = 2*pi + x
#
#    if (x.ub > pi)&(x.lb < -pi):
#        return Interval(-1,1)
#    elif (x.ub > pi):
#        if(x.lb>pi/2):
#            Ub = max(sin(x.lb),sin(x.ub))
#            Lb = min(sin(x.lb),sin(x.ub))
#            return Interval(Lb,Ub)
#        else:
#            Ub = 1
#            Lb = min(sin(x.lb),sin(x.ub))
#            return Interval(Lb,Ub)
#    elif  (x.lb < -pi):
#        if (x.ub < -pi/2):
#            Ub = sin(x.lb)
#            Lb = sin(x.ub)
#            return Interval(Lb,Ub) 
#        else:
#            Lb = -1
#            Ub = max(sin(x.lb),sin(x.ub))
#            return Interval(Lb,Ub)
#    
#    if (x.ub > pi/2)&(x.lb > pi/2):
#        Lb = min(sin(x.lb),sin(x.ub))
#        Ub = max(sin(x.lb),sin(x.ub))
#        return Interval(Lb,Ub)
#    
#    if (x.ub < -pi/2)&(x.ub < -pi/2):
#        Lb = min(sin(x.lb),sin(x.ub))
#        Ub = max(sin(x.lb),sin(x.ub))
#        return Interval(Lb,Ub)
#    
#    if pi/2 in x:
#        Ub = 1
#        if -pi/2 in x:
#            Lb = -1
#        else:
#            Lb = min(sin(x.lb),sin(x.ub))
#    elif -pi/2 in x:
#        Lb = -1
#        Ub = max(sin(x.lb),sin(x.ub))
#    else:
#        Lb = sin(x.lb)
#        Ub = sin(x.ub)
#
#    return Interval(Lb,Ub)



def sign(x):
    return Interval(np.sign(x.lb),np.sign(x.ub))

def abs(x):

    if x.ub <= 0:
        absx = Interval(np.abs(x.ub),np.abs(x.lb))
    elif 0 in x:
        absx = Interval(0,np.max([np.abs(x.lb),np.abs(x.ub)]))
    else:
        absx = x

    return absx



class IntervalVector:

	def __init__(x,a):
		x.b1 = Interval(a[0][0],a[0][1])
		x.b2 = Interval(a[1][0],a[1][1])

	def __getitem__(x,a):
        	if a == 0:
	            	return x.b1
        	elif a == 1:
        	    	return x.b2

    	def __setitem__(x,c,d) :
        	if c == 0:
            		a = d
            		b = x.b2

        	elif c == 1:
            		a = x.b1
            		b = d

            	x.b1, x.b2 = a, b


	def __repr__(x):
		return ('[[%f,%f],[%f,%f]]'%(x.b1.lb,x.b1.ub,x.b2.lb,x.b2.ub))



