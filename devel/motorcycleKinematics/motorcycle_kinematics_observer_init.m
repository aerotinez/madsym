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
    0.0882;
    0.6429;
    0.4189;
    0.0253;
    0.2344;
    0.5750;
    0.282;
    0.297;
    ];

%% Generalized coordinates
yaw = T.Yaw;
pitch = T.Pitch;
roll = T.Roll_E;
[yaw,lean,pitch] = eulZYXToZXY(yaw,pitch,roll);
steer = T.Steer;
eul = [yaw,lean,pitch,steer];
q = timeseries(deg2rad(eul),t);
q0 = [q.Data(1,:),0];

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
nx = 5;
nz = 9;
P0 = eye(nx);
Q = eye(nx);
R = eye(nz);
