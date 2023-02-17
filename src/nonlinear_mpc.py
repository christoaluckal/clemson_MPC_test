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
import matplotlib.pyplot as plt

silverstone_length = 457.1467131573973
osch = 259.65211118046517
ims = 0
global_speed = 7

# Define necessary vehicle parameters
WB = 0.324 # Wheelbase of the vehicle
N_x = 4 # Number of state for MPC design
N_u = 2 # Number of control for MPC design
vel_max = 15 # Set the maximum velocity
vel_min = 0.1 # Set the minimum velocity
max_steer = 0.4189 # Set the maximum steering angle
max_accln = 100

N = 5 # Number of steps in the interval

 # Time interval
time = silverstone_length/global_speed
# time = 80 # Time to complete the raceline
future_time = 2 # Future Look Ahead time
dt = future_time/N
# dt = 1

qx = float(sys.argv[2]) # State error scale for x_pos
qy = float(sys.argv[3]) # State error scale for y_pos
q_yaw = float(sys.argv[4]) # State error scale for yaw
q_vel = float(sys.argv[5]) # State error scale for velocity
r_acc = float(sys.argv[6]) # Input cost of accln
r_steer = float(sys.argv[7]) # Input cost of steer
u_steer = float(sys.argv[9]) # Steering constraints
u_acc = float(sys.argv[8]) # Accln constraints
trial = int(sys.argv[10])

class ros_message_listener:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vel = 0.0
        self.x_vel = 0
        self.y_vel = 0
        self.odom = []
        self.odom_sub = rospy.Subscriber('/car_1/base/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vel = math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
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
        self.x_vel = msg.twist.twist.linear.x
        self.y_vel = msg.twist.twist.linear.y
        self.yaw = yaw
        # self.odom.append([self.x,self.y,self.x_vel,self.y_vel,self.yaw])
        self.odom.append([self.x,self.y,self.x_vel,self.y_vel])

    def vehicle_state_output(self):
        vehicle_state = np.array([self.x, self.y, self.yaw, self.vel,self.x_vel,self.y_vel])
        return vehicle_state

def nonlinear_kinematic_mpc_solver(x_ref, x_0, N):
    opti = casadi.Opti()

    x = opti.variable(N_x, N+1)
    u = opti.variable(N_u, N)
    x_ref = casadi.MX(x_ref)
    x_0 = casadi.MX(x_0)

    Q = casadi.diag([qx, qy, q_yaw, q_vel])
    R = casadi.diag([r_acc, r_steer])
    cost = 0

    for t in range(N-1):
        cost += u[:, t].T @ R @ u[:, t]
        if t != 0:
            cost += (x_ref[:, t].T - x[:, t].T) @ Q @ (x_ref[:, t] - x[:, t])
        opti.subject_to(x[0, t + 1] == x[0, t] + x[3, t]*casadi.cos(x[2, t])*dt)
        opti.subject_to(x[1, t + 1] == x[1, t] + x[3, t]*casadi.sin(x[2, t])*dt)
        opti.subject_to(x[2, t + 1] == x[2, t] + x[3, t]*casadi.tan(u[1, t])*dt/WB)
        opti.subject_to(x[3, t + 1] == x[3, t] + u[0, t]*dt)

        if t < N-2:
            opti.subject_to(u[1, t+1] - u[1, t] >= -u_steer)
            opti.subject_to(u[1, t+1] - u[1, t] <= u_steer)
            opti.subject_to(u[0, t+1] - u[0, t] <= u_acc)
            opti.subject_to(u[0, t+1] - u[0, t] >= -u_acc)

        # if t < N-2:
        #     opti.subject_to(u[1, t+1] - u[1, t] >= -0.06)
        #     opti.subject_to(u[1, t+1] - u[1, t] <= 0.06)
        #     opti.subject_to(u[0, t+1] - u[0, t] <= 0.1)
        #     opti.subject_to(u[0, t+1] - u[0, t] >= -0.1)

    opti.subject_to(x[:, 0] == x_0)
    opti.subject_to(u[1, :] <= max_steer)
    opti.subject_to(u[1, :] >= -max_steer)
    opti.subject_to(u[0, :] <= max_accln)
    opti.subject_to(u[0, :] >= -max_accln)

    opti.minimize(cost)
    opti.solver('ipopt',{"print_time": False}, {"print_level": 0})#, {"acceptable_tol": 0.0001}
    sol = opti.solve()

    acceleration = sol.value(u[0,0])
    steering = sol.value(u[1,0])

    return acceleration, steering



def rviz_markers(pose,idx):
    if idx == 0:
        points = Marker()
        points.type = Marker.POINTS
        points.header.frame_id = "map"
        points.ns = "raceline"
        points.action = Marker.ADD
        points.pose.orientation.w = 1
        points.scale.x = 0.5
        points.scale.y = 0.5
        points.color.r = 0.5
        points.color.g = 0.5
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
        points.scale.x = 1
        points.scale.y = 1
        points.color.b = 1
        points.color.a = 1

    for i in pose:
        p = Point()
        p.x = i[0]
        p.y = i[1]
        points.points.append(p)

    return points


def reference_pose_selection(x_spline,y_spline, curr_t,N):
    delta_t = future_time
    while curr_t+delta_t > time:
        delta_t-=dt

    if delta_t<0.01:
        curr_t = 0
        delta_t = future_time
    
    t_vec = np.linspace(curr_t,curr_t+delta_t,N)
    xTraj = x_spline(t_vec)
    yTraj = y_spline(t_vec)

    xdTraj = x_spline(t_vec,1)
    ydTraj = y_spline(t_vec, 1)

    thTraj = np.arctan2(ydTraj,xdTraj)
    
    vTraj = np.sqrt(np.power(xdTraj,2)+ np.power(ydTraj,2))
    path_array = np.array([xTraj,yTraj,thTraj,vTraj]).T

    return path_array

def write_csv(time_l,x_ref,x_pos,y_ref,y_pos,speeds,acc,phi,x_err,y_err,x_dot_err,y_dot_err,odoms):
    
    track = 'silverstone'
    # trial_num = 1
    folder = 'trials/'+track+'/speed_'+str(global_speed)
    import os
    try: 
        os.mkdir(folder) 
    except OSError as error:
        pass 

    fig, axs = plt.subplots(1, 3)
    axs[0].plot(x_pos,y_pos,'--',color='orange',label='executed')
    axs[0].plot(global_path[:,0],global_path[:,1],label='reference')
    axs[0].set_title('Speed:{} m/s'.format(global_speed))

    axs[1].plot(time_l,phi,color='orange',label='phi')
    axs[1].plot(time_l,acc,label='acceleration')
    axs[1].plot(time_l,speeds,'--',color='green',label='speed')

    axs[2].plot(time_l,x_err,label='x error')
    axs[2].plot(time_l,y_err, color='orange',label='y error')
    axs[2].plot(time_l,x_dot_err, color='green',label='x dot error')
    axs[2].plot(time_l,y_dot_err, color='red',label='y dot error')
    axs[0].grid()
    axs[0].legend()
    axs[1].grid()
    axs[1].legend()
    axs[2].grid()
    axs[2].legend()
    plt.plot()
    plt.show()
    # plt.savefig(folder+'/trial_{}.png'.format(trial))

    print(folder)
    with open(folder+'/trial_{}_metrics.csv'.format(trial),'w+') as csv_f:
        csv_f.write('N:{},future:{},qx:{},qy:{},q_yaw:{},q_vel:{},r_acc:{},r_steer:{},u_steer:{},u_acc:{}\n\n\n'.format(N,future_time,qx,qy,q_yaw,q_vel,r_acc,r_steer,u_steer,u_acc))
        csv_f.write('time,x_ref,x_pos,y_ref,y_pos,speed,acc,phi,x_err,y_err,x_dot_err,y_dot_err\n')
        for i in range(len(time_l)):
            t = time_l[i]
            xr = x_ref[i]
            x = x_pos[i]
            yr = y_ref[i]
            y = y_pos[i]
            s = speeds[i]
            a = acc[i]
            p = phi[i]
            xe = x_err[i]
            ye = y_err[i]
            xd_e = x_dot_err[i]
            yd_e = y_dot_err[i]
            csv_f.write('{},{},{},{},{},{},{},{},{},{},{},{}\n'.format(t,xr,x,yr,y,s,a,p,xe,ye,xd_e,yd_e))

    with open(folder+'/trial_{}_odom.csv'.format(trial),'w+') as csv_f:
        csv_f.write('x,y,x_vel,y_vel\n')
        for i in odoms:
            csv_f.write('{},{},{},{}\n'.format(i[0],i[1],i[2],i[3]))

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

    rate = rospy.Rate(100)
    vehicle_pose_msg = ros_message_listener()
    # N = 5

    ref_list = np.array(global_path[:,0:2])
    x_pos = []
    x_ref_pos = []
    y_pos = []
    y_ref_pos = []
    speeds = []
    acc = []
    phi = []
    x_err = []
    y_err = []
    x_dot_err = []
    y_dot_err = []
    time_l = []
    init_time = rospy.get_time()
    delta_t = 0
    speed_ref = []
    odoms = []
    while not rospy.is_shutdown():
        try:
            curr_t = rospy.get_time() - init_time
            delta_t = rospy.get_time() - curr_t
            current_state = vehicle_pose_msg.vehicle_state_output()
            # rospy.loginfo(curr_t)
            
            # Transform the reference pose to vehicle coordinate
            # if curr_t > future_time:
            if True:
                reference_pose = reference_pose_selection(x_spline,y_spline, curr_t, N)
                x_ref = np.zeros((N, 4))
                for i in range(N):
                    x, ref = vehicle_coordinate_transformation(reference_pose[i,:], current_state)
                    x_ref[i, :] = ref
                
                # Compute Control Output from Nonlinear Model Predictive Control
                acceleration, steering = nonlinear_kinematic_mpc_solver(x_ref.T, x.T, N)
                
                # speed = np.clip(current_state[3] + acceleration*dt, vel_min, vel_max)
                speed = current_state[3]+acceleration*dt
                # print(current_state[3],speed)
                drive_msg = AckermannDrive()
                drive_msg.speed = speed
                drive_msg.steering_angle = steering
                drive_pub.publish(drive_msg)

                # if delta_t > dt:

                x_pos.append(current_state[0])
                y_pos.append(current_state[1])

                speeds.append(speed)

                acc.append(acceleration)

                phi.append(steering)

                x_r_curr = x_spline(curr_t)
                x_ref_pos.append(x_r_curr)
                x_err.append(x_r_curr-current_state[0])

                y_r_curr = y_spline(curr_t)
                y_ref_pos.append(y_r_curr)
                y_err.append(y_r_curr-current_state[1])

                speed_ref.append(math.sqrt(x_spline(curr_t,1)**2+y_spline(curr_t,1)**2))

                x_dot_err.append(x_spline(curr_t,1)-current_state[4])
                y_dot_err.append(y_spline(curr_t,1)-current_state[5])

                time_l.append(curr_t)


            raceline_pub.publish(rviz_markers(global_path,0))
            spline_marker_pub.publish(rviz_markers(reference_pose,1))
            rate.sleep()
        except IndexError:
            continue
        except RuntimeError:
            continue
        except rospy.exceptions.ROSInterruptException:
            break
        

    save = input('Save?')
    if save == 'y':
        # plt.plot(time_l,speeds)
        # plt.plot(time_l,speed_ref,color='red')
        # plt.show()
        write_csv(time_l,x_ref_pos,x_pos,y_ref_pos,y_pos,speeds,acc,phi,x_err,y_err,x_dot_err,y_dot_err,vehicle_pose_msg.odom)

        # fig, axs = plt.subplots(1, 3)
        # axs[0].plot(x_pos,y_pos,'--',color='orange',label='executed')
        # axs[0].plot(global_path[:,0],global_path[:,1],label='reference')

        # axs[1].plot(time_l,phi,color='orange',label='phi')
        # axs[1].plot(time_l,acc,label='acceleration')
        # axs[1].plot(time_l,speeds,'--',color='green',label='speed')

        # axs[2].plot(time_l,x_err,label='x error')
        # axs[2].plot(time_l,y_err, color='orange',label='y error')
        # axs[2].plot(time_l,x_dot_err, color='green',label='x dot error')
        # axs[2].plot(time_l,y_dot_err, color='red',label='y dot error')
        # axs[0].grid()
        # axs[0].legend()
        # axs[1].grid()
        # axs[1].legend()
        # axs[2].grid()
        # axs[2].legend()
        # plt.show()
    
