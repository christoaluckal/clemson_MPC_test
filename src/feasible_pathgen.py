from scipy.interpolate import interp1d, CubicSpline
import numpy as np
import math
import pandas as pd

def get_spline_path(csv_f,vmax):
    # ref = pd.read_csv(f"scripts/{track}_centerline_reference_Optimal.csv")
    ref = pd.read_csv(csv_f)

    coords = ref[["x_m", "y_m"]].to_numpy()
    tvec = ref[[f"Tvec_{vmax}"]].to_numpy().reshape(coords.shape[0])
    tmax= tvec[-1]
    xCoords = coords[:,0]
    yCoords = coords[:,1]

    track_distance = 0
    for i in range(1,len(xCoords)-1):
        track_distance += math.sqrt((xCoords[i+1]-xCoords[i])**2+(yCoords[i+1]-yCoords[i])**2)

    print("Track Length:",track_distance)
    # tvec = np.linspace(0,time,len(xCoords))
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

    return (path_array,xTrajCS,yTrajCS, tmax)

