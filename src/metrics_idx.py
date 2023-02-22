import sys
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import numpy as np
import math
csv_f = open(sys.argv[1])

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
for i in csv_f.readlines()[5:]:
    vals = i.split(',')
    z.append(0)
    t.append(float(vals[0]))
    x.append(float(vals[1]))
    y.append(float(vals[2]))
    xr.append(float(vals[3]))
    yr.append(float(vals[4]))


refs = np.array([xr,yr,z])
refs = refs.T

actual = np.array([x,y,z])
actual = actual.T

weights = np.ones((len(refs)))/(len(refs))

rot,rmsd = Rotation.align_vectors(refs,actual,weights)
print("KABSH:",rmsd)
# plt.plot(xr,yr)
# plt.plot(x,y)
# plt.show()
# plt.plot(t,s)
# plt.plot(t,xe)
# plt.plot(t,ye)
# plt.show()
sd = 0
for i in range(len(t)):
    sd+= math.sqrt((xr[i]-x[i])**2+(yr[i]-y[i])**2)

rmsd = sd/len(t)

print("REG:",rmsd)