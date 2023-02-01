from scipy.interpolate import interp1d, CubicSpline
import numpy as np


def get_spline_path(csv_f,time):
    path = csv_f

    waypoints = np.genfromtxt(path, dtype=float, delimiter=",")
    xCoords = waypoints[:,0]
    yCoords = waypoints[:,1]


    tvec = np.linspace(0,time,len(xCoords))
    xTrajCS = CubicSpline(tvec,xCoords)
    yTrajCS = CubicSpline(tvec,yCoords)

    xTraj = xTrajCS(tvec)
    yTraj = yTrajCS(tvec)

    xdTraj = xTrajCS(tvec,1)
    ydTraj = yTrajCS(tvec, 1)

    xddTraj = xTrajCS(tvec,2)
    yddTraj = yTrajCS(tvec, 2)

    thTraj = np.arctan2(ydTraj,xdTraj)
    thdTraj = np.divide(np.multiply(yddTraj, xdTraj)-np.multiply(ydTraj, xddTraj), (np.power(xdTraj,2)+np.power(ydTraj,2)))

    vTraj = np.sqrt(np.power(xdTraj,2)+ np.power(ydTraj,2))

    path_array = np.array([xTraj,yTraj,thTraj,vTraj]).T

    return (path_array,xTrajCS,yTrajCS)

