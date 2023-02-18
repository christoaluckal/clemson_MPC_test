# clemson_MPC_test

Based on : https://github.com/rongyaw/f1tenth-clemson

Run using

`python3 nonlinear_mpc.py track_letter q_x q_y q_yaw q_vel r_acc r_steer u_acc u_steer`

| Param  | Context |
| ------------- | ------------- |
| track_letter | i,m,s,o |
| q_x  | State cost of x position  |
| q_y  | State cost of y position  |
| q_yaw  | State cost of yaw  |
| q_vel  | State cost of velocity  |
| r_acc  | Control cost of acceleration  |
| r_steer  | Control cost of steer  |
| u_acc  | Acceleration constraint  |
| u_steer  | Steering constraint  |

i: IMS <br>
m: Monza <br>
s: Silverstone <br>
o: Oschersleben <br>

Generally higher r_* values means controller will not erratically change states but won't converge <br>
Higher q_* values means controller will try to converge quickly and erratically <br>
u_* values are the change in acc/steering values. Higher values means controller can give larger actuation values <br>
