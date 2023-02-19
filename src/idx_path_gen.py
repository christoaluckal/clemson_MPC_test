from scipy.interpolate import interp1d, CubicSpline
import numpy as np
import math

def get_spline_path(csv_f,time):
    path = csv_f

    waypoints = np.genfromtxt(path, dtype=float, delimiter=",")
    xCoords = waypoints[1:,1]
    yCoords = waypoints[1:,0]

    correction_angle = -math.atan2(yCoords[1]-yCoords[0],xCoords[1]- xCoords[0])
    R_z = np.array(
                    [[math.cos(correction_angle), -math.sin(correction_angle)],
                    [math.sin(correction_angle), math.cos(correction_angle)]])
    coords = zip(xCoords, yCoords)
    corrected_xCoords = []
    corrected_yCoords = []

    for p in coords:
        p = np.matmul(R_z,np.array(p).T)
        corrected_xCoords.append(p[0])
        corrected_yCoords.append(p[1])

    # xCoords = corrected_xCoords
    # yCoords = corrected_yCoords

    track_distance = 0
    for i in range(1,len(xCoords)-1):
        track_distance += math.sqrt((xCoords[i+1]-xCoords[i])**2+(yCoords[i+1]-yCoords[i])**2)

    tmax = time
    print("Track Length:",track_distance)
    tvec = np.linspace(0,tmax,len(xCoords))
    xTrajCS = CubicSpline(tvec,corrected_xCoords)
    yTrajCS = CubicSpline(tvec,corrected_yCoords)

    tvec = np.linspace(0,tmax,1000)

    x_ref = np.array(xTrajCS(tvec)).reshape(-1,1)
    y_ref = np.array(yTrajCS(tvec)).reshape(-1,1)

    x_refd = np.array(xTrajCS(tvec,1)).reshape(-1,1)
    y_refd = np.array(yTrajCS(tvec,1)).reshape(-1,1)

    x_refdd = np.array(xTrajCS(tvec,2)).reshape(-1,1)
    y_refdd = np.array(yTrajCS(tvec,2)).reshape(-1,1)

    thTraj = np.arctan2(y_refd,x_refd).reshape(-1,1)
    # thdTraj = np.divide(np.multiply(yddTraj, x_refd)-np.multiply(y_refd, xddTraj), (np.power(x_refd,2)+np.power(y_refd,2)))

    vTraj = np.sqrt(np.power(x_refd,2)+ np.power(y_refd,2)).reshape(-1,1)
    s0 = np.zeros((1,4))
    # path_array = np.array([x_ref,y_ref,thTraj,vTraj]).T
    ref = np.hstack((x_ref, y_ref, thTraj, vTraj))
    ref = np.insert(ref,[0], s0, axis=0)
    return (ref)

if __name__ == "__main__":
    print(get_spline_path("src/track_csvs/Silverstone_centerline.csv", 120).shape)

