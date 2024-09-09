close("all"); clear; clc;
setmadsympath();
ts = 1/60;

%% Bikesim dataset
vx = 130;
dir_str = "G:\My Drive\BikeSimResults\BigSports\ICRA25\Vx" + vx + "Kph\";
file_str = "bikesim_results_" + vx +"kph.csv";
T = readtable(dir_str + file_str);
t = T.Time;
tf = t(end);

%% Geometric parameters
params = [
    0.7525;
    0.6429;
    0.4189;
    0.0253;
    0.2344;
    0.5750
    ];

%% Generalized coordinates
yaw = T.Yaw;
pitch = T.Pitch;
roll = T.Roll_E;
steer = T.Steer;
eul = [yaw,pitch,roll,steer];
q = timeseries(deg2rad(eul),t);
q0 = q.Data(1,:);

%% Generalized speeds
yaw_rate = T.AV_Y;
pitch_rate = T.AV_P;
roll_rate = T.AV_R;
steer_rate = T.M_StrSys./0.2122;
euld = [yaw_rate,pitch_rate,roll_rate,steer_rate];
qd = timeseries(deg2rad(euld),t);

%% Steering frame velocities
wx = T.AVx;
wy = T.AVy;
wz = T.AVz;
vx = T.Vx_S1;
vy = T.Vy_S1;
vz = T.Vz_S1;
w = [wx,wy,wz];
v = [vx,vy,vz];
u = timeseries([deg2rad(w),v./3.6],t);

%% EKF covariance matrices
nx = 4;
nz = 9;
P0 = eye(nx);
Q = 1E-03.*eye(nx);
R = eye(nz);
