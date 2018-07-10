#!/usr/bin/env python
# license removed for brevity

from math import pi
from numpy import array
import numpy as np


# sailboat parameter

p1 = 0.03
p2 = 40.0
p3 = 6000.0
p4 = 200.0
p5 = 1500.0
p6 = 0.5
p7 = 0.5
p8 = 2.0
p9 = 300
p10 = 400
p11 = 0.2
p100 = p1

param = [p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p100]


# Keeping position parameters

dsecu2 = 15
dsecu = 30  # dsecu2 + pi*p3/np.abs(p5*p8*np.sin(2*pi/4))
rb = (p8)/np.sin(pi/4)	# turning angle
darret0 = 2*rb

dsecuObs = 2*p8  + pi*p3/np.abs(p5*p8*np.sin(2*pi/4))


# Sail control

kp = 1
kv = 8
ka = 5

# Other

deltarmax = pi/4
delta = pi/4


