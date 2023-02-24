import sys
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import numpy as np
import math
from sklearn.metrics import mean_squared_error
import os




def getmse(track):
    if track == 'i':
        track_name = 'IMS'
    elif track == 'm':
        track_name = 'Monza'
    elif track == 's':
        track_name = 'Silverstone'
    elif track == 'o':
        track_name = 'Oschersleben'

    t = []
    xr = []
    x = []
    yr = []
    y = []
    z = []
    s= []
    a = []
    xe = []
    ye = []
    speeds = ['8','10','12']
    for sp in speeds:
        for tr in range(1,6):
            csv_file = 'trials/feasible{}/{}/{}_{}_trial_{}.csv'.format(track_name,sp,track_name,sp,tr)
            csv_f = open(csv_file,'r')
            for i in csv_f.readlines()[1:]:
                vals = i.split(',')
                z.append(0)
                t.append(float(vals[0]))
                xr.append(float(vals[1]))
                x.append(float(vals[2]))
                yr.append(float(vals[3]))
                y.append(float(vals[4]))
                s.append(float(vals[5]))
                a.append(float(vals[6]))
                xe.append(float(vals[8]))
                ye.append(float(vals[9]))

            refs = np.array([xr,yr,z])
            refs = refs.T

            actual = np.array([x,y,z])
            actual = actual.T

            weights = np.ones((len(refs)))/(len(refs))

            rot,rmsd = Rotation.align_vectors(refs,actual,weights)

            # plt.plot(xr,yr)
            # plt.plot(x,y)
            # plt.show()
            # plt.plot(t,s)
            # plt.plot(t,xe)
            # plt.plot(t,ye)
            plt.plot()
            # plt.show()
            sd = 0
            for i in range(len(t)):
                sd+= math.sqrt((xr[i]-x[i])**2+(yr[i]-y[i])**2)

            rmsd = sd/len(t)

            print("REG:",rmsd)
            print("{}_{}_{}_SK:".format(track_name,sp,tr),mean_squared_error(actual,refs))
            print('\n\n')


getmse('i')
getmse('m')
getmse('s')
getmse('o')