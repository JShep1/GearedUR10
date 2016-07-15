import os,sys
import pylab
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as lines

############################
# TODO: set values here 
# see figure 1 in homework for definition of variables
set_point = 0.0 
rise_time = 0.02
peak_time = 0.01
overshoot = 0.02
settling_time = 0.1

# Error bands are at y = (1 +/- delta)
error_band_delta = 0.02

# offset for text
eps = .05
############################

# check proper number of arguments given
if len(sys.argv) < 2:
  print 'syntax: plotGearJoints.py <joint name>'
  sys.exit(0)  

# setup font
font =  {'family' : 'sans-serif',
         'color'  : 'black',
         'weight' : 'normal',
         'size'   : 16,
        }

# step size (constant value)
DT05   = 0.05
DT01   = 0.01
DT001 = 0.001
# load data
y2 = pylab.loadtxt(sys.argv[1] + "_001.txt")
y3 = pylab.loadtxt(sys.argv[1] + "_01.txt")
y4 = pylab.loadtxt(sys.argv[1] + "_05.txt")

t05  = np.linspace(0,  y4.size*DT05,y4.size)
t01  = np.linspace(0, y3.size*DT01,y3.size)
t001 = np.linspace(0,y2.size*DT001,y2.size)

# setup the figure
fig = plt.figure()

#print "%d" % y3.size

# plot data
plt.plot(t001,y2,'r' ,label='Error-0.001')
plt.plot(t01, y3,'g' ,label='Error-0.01')
plt.plot(t05, y4,'b' ,label='Error-0.05')

# set limits
axPlot = plt.subplot(111)
axPlot.set_xlim(-.001, 3)
s = 'Positional Error for Different Timesteps';
# add titles, labels, and legend
plt.title(s, fontdict=font)
plt.xlabel('Time', fontdict=font)
plt.ylabel('Error', fontdict=font)
plt.legend(loc=4, shadow=True)

# annotate rise time (interval)
plt.axhspan(set_point-error_band_delta,set_point+error_band_delta, facecolor='0.5', alpha=0.25)
plt.axhline(y=set_point-error_band_delta, color='k',linestyle='--')
plt.axvspan(0.0,rise_time,facecolor='0.5', alpha=0.25)
#plt.axvline(x=rise_time, color='k',linestyle='--')
#plt.annotate('rise time', xy=(rise_time-.5, set_point-eps))

# annotate peak_time
#plt.axvline(x=peak_time, color='k',linestyle='--')
#plt.annotate('peak time', xy=(peak_time+.1, set_point-2*eps))

# annotate overshoot
#plt.axhline(y=overshoot, xmin=0, xmax=peak_time,color='k',linestyle='--')
#plt.annotate('overshoot', xy=(peak_time, overshoot+eps))

# annotate settling time
#plt.axvline(x=settling_time,color='k',linestyle='--')
#plt.annotate('settling time', xy=(settling_time, set_point+eps))

# draw error band
#plt.axhspan(set_point-error_band_delta,set_point+error_band_delta, facecolor='0.5', alpha=0.25)
#plt.axhline(y=set_point-error_band_delta, color='k',linestyle='--')
#plt.axhline(y=set_point+error_band_delta, color='k',linestyle='--')
#plt.plot([0, y1[:,0].size*DT], [-0.5, -0.5], '--', lw=1.0)
#plt.plot([0, y1[:,0].size*DT], [-1.5, -1.5], '--', lw=1.0)
#plt.annotate('error band', xy=(10.1, set_point))

# show the plot
#plt.show()

# save the plot
plt.savefig(sys.argv[1] + '_error.png')

