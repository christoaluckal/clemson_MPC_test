#!/usr/bin/env python3

'''
Topic: Kinematic Nonlinear Model Predictive Controller for F1tenth simulator
Author: Rongyao Wang
Instiution: Clemson University Mechanical Engineering
'''

import rospy
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PoseStamped, PoseArray, Pose,Point
from visualization_msgs.msg import Marker
import math
from utils import yaw_change_correction, vehicle_coordinate_transformation, goal_reach_check, global_path_reader, nearest_reference_pose
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import casadi
from timeit import default_timer as timer
import csv
import os
import path_gen
import sys

# Define necessary vehicle parameters
WB = 0.324 # Wheelbase of the vehicle
N_x = 4 # Number of state for MPC design
N_u = 2 # Number of control for MPC design
N = 10 # Number of steps in the interval
vel_max = 15 # Set the maximum velocity
vel_min = 0.1 # Set the minimum velocity
max_steer = 0.4189 # Set the maximum steering angle

dt = 0.05 # Time interval
time = 100 # Trial time

# q_d1 = float(sys.argv[2])
# q_d2 = float(sys.argv[3])
# r_d = float(sys.argv[4])
qx = float(sys.argv[2])
qy = float(sys.argv[3])
q_yaw = float(sys.argv[4])
q_vel = float(sys.argv[5])

class ros_message_listener:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vel = 0.0
        self.odom_sub = rospy.Subscriber('/car_1/base/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vel = msg.twist.twist.linear.x
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        quaternion = (qx,qy,qz,qw)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]
        self.x = x
        self.y = y
        self.vel = vel
        self.yaw = yaw

    def vehicle_state_output(self):
        vehicle_state = np.array([self.x, self.y, self.yaw, self.vel])
        return vehicle_state

def nonlinear_kinematic_mpc_solver(x_ref, x_0, N):
    opti = casadi.Opti()

    x = opti.variable(N_x, N+1)
    u = opti.variable(N_u, N)
    x_ref = casadi.MX(x_ref)
    x_0 = casadi.MX(x_0)

    Q = casadi.diag([q_d1, q_d1, q_d2, q_d2])
    R = casadi.diag([r_d, r_d])
    cost = 0

    for t in range(N-1):
        cost += u[:, t].T @ R @ u[:, t]
        if t != 0:
            cost += (x_ref[:, t].T - x[:, t].T) @ Q @ (x_ref[:, t] - x[:, t])
        opti.subject_to(x[0, t + 1] == x[0, t] + x[3, t]*casadi.cos(x[2, t])*dt)
        opti.subject_to(x[1, t + 1] == x[1, t] + x[3, t]*casadi.sin(x[2, t])*dt)
        opti.subject_to(x[2, t + 1] == x[2, t] + x[3, t]*casadi.tan(u[1, t])/WB*dt)
        opti.subject_to(x[3, t + 1] == x[3, t] + u[0, t]*dt)

        if t < N-2:
            opti.subject_to(u[1, t+1] - u[1, t] >= -0.06)
            opti.subject_to(u[1, t+1] - u[1, t] <= 0.06)
            opti.subject_to(u[0, t+1] - u[0, t] <= 0.1)
            opti.subject_to(u[0, t+1] - u[0, t] >= -0.1)

    opti.subject_to(x[:, 0] == x_0)
    opti.subject_to(u[1, :] <= max_steer)
    opti.subject_to(u[1, :] >= -max_steer)
    opti.subject_to(u[0, :] <= 1)
    opti.subject_to(u[0, :] >= -1)

    opti.minimize(cost)
    opti.solver('ipopt',{"print_time": False}, {"print_level": 0})#, {"acceptable_tol": 0.0001}
    sol = opti.solve()

    acceleration = sol.value(u[0,0])
    steering = sol.value(u[1,0])

    return acceleration, steering

# def reference_pose_selection(global_path, reference_pose_id, distance_interval, N, num_path):
#     distance = 0.0
#     i = reference_pose_id + 1
#     reference_pose = np.zeros((N, 4))
#     for k in range(N):
#         while distance < distance_interval:
#             if i < num_path:
#                 distance = ((global_path[i,0] - global_path[reference_pose_id,0])**2 + (global_path[i,1] - global_path[reference_pose_id,1])**2)**0.5
#                 i += 1
#             else:
#                 i -= num_path
#                 distance = ((global_path[i,0] - global_path[reference_pose_id,0])**2 + (global_path[i,1] - global_path[reference_pose_id,1])**2)**0.5
#                 i += 1
#         reference_pose[k, :] = np.array([global_path[i,0], global_path[i,1], global_path[i,2], global_path[i,3]])
#         reference_pose_id = i
#         distance = 0.0
#     return reference_pose

def rviz_markers(pose,idx):
    if idx == 0:
        points = Marker()
        points.type = Marker.POINTS
        points.header.frame_id = "map"
        points.ns = "raceline"
        points.action = Marker.ADD
        points.pose.orientation.w = 1
        points.scale.x = 0.05
        points.scale.y = 0.05
        points.color.r = 1
        points.color.a = 1

        for i in pose:
            p = Point()
            p.x = i[0]
            p.y = i[1]
            points.points.append(p)

    elif idx == 1:
        points = Marker()
        points.type = Marker.POINTS
        points.header.frame_id = "map"
        points.ns = "spline"
        points.action = Marker.ADD
        points.pose.orientation.w = 1
        points.scale.x = 0.1
        points.scale.y = 0.1
        points.color.b = 1
        points.color.a = 1

    for i in pose:
        p = Point()
        p.x = i[0]
        p.y = i[1]
        points.points.append(p)

    return points


def reference_pose_selection(x_spline,y_spline, curr_t,N):
    delta_t = 1
    if curr_t+delta_t > time:
        curr_t = 0
    
    t_vec = np.linspace(curr_t,curr_t+delta_t,N)
    xTraj = x_spline(t_vec)
    yTraj = y_spline(t_vec)

    xdTraj = x_spline(t_vec,1)
    ydTraj = y_spline(t_vec, 1)

    thTraj = np.arctan2(ydTraj,xdTraj)
    
    vTraj = np.sqrt(np.power(xdTraj,2)+ np.power(ydTraj,2))
    path_array = np.array([xTraj,yTraj,thTraj,vTraj]).T

    return path_array

def goal_pose_publisher(goal_pose, N):
    frame_id = 'map'
    msg = PoseArray()
    msg.header.frame_id = frame_id
    for i in range(N):
        x = goal_pose[i,0]
        y = goal_pose[i,1]
        yaw = goal_pose[i,2]
        quaternion = quaternion_from_euler(0.0, 0.0, yaw)
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        msg.poses.append(pose)
    return msg

if __name__ == "__main__":
    rospy.init_node('nonlinear_mpc')
    
    drive_pub = rospy.Publisher('car_1/command', AckermannDrive, queue_size=1)
    raceline_pub = rospy.Publisher('visualization_markers',Marker,queue_size=1)
    spline_marker_pub = rospy.Publisher('visualization_markers',Marker,queue_size=1)

    csv_f = sys.argv[1]

    global_path,x_spline,y_spline = path_gen.get_spline_path(csv_f,time)

    num_path = len(global_path[:,0])
    
    rate = rospy.Rate(100)
    vehicle_pose_msg = ros_message_listener()
    # N = 5

    init_time = rospy.get_time()
    while not rospy.is_shutdown():
        try:
            curr_t = rospy.get_time() - init_time
            current_state = vehicle_pose_msg.vehicle_state_output()
            
            reference_pose = reference_pose_selection(x_spline,y_spline, curr_t, N)
            # Transform the reference pose to vehicle coordinate
            x_ref = np.zeros((N, 4))
            for i in range(N):
                x, ref = vehicle_coordinate_transformation(reference_pose[i,:], current_state)
                x_ref[i, :] = ref
            
            # Compute Control Output from Nonlinear Model Predictive Control
            acceleration, steering = nonlinear_kinematic_mpc_solver(x_ref.T, x.T, N)
            speed = np.clip(current_state[3] + acceleration, vel_min, vel_max)
            
            drive_msg = AckermannDrive()
            drive_msg.speed = speed + 1.0
            drive_msg.steering_angle = steering
            drive_pub.publish(drive_msg)
            raceline_pub.publish(rviz_markers(global_path,0))
            spline_marker_pub.publish(rviz_markers(reference_pose,1))
        except IndexError:
            continue
        except RuntimeError:
            continue
        rate.sleep()
