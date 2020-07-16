#!/usr/bin/python

import roslib; roslib.load_manifest('fmutil')
from fmutil import LowPassFilter

import numpy as np
import matplotlib.pyplot as plt


npts = 10000
dt = np.random.random(npts-1)*0.01+0.001

dt[4000] = 1

t = np.zeros(npts)
for i in range(1,npts):
    t[i] = t[i-1] + dt[i-1]

x  = np.sin(t*5)
xn = x + (np.random.random(npts)-0.5)*0.5

f = LowPassFilter(0.1)
y = np.zeros(npts)
for i in range(npts):
    y[i] = f.filter(t[i],xn[i])

plt.figure()
plt.plot(t,x,'r', t,xn,'b', t,y,'g')
plt.show()