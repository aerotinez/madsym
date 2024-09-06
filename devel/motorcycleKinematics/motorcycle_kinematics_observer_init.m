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
px = T.X_S2;
py = T.Y_S2;
pz = T.Z_S2;
eul = [yaw,pitch,roll,steer];
p = [px,py];
q = timeseries([deg2rad(eul),p],t);
q0 = q.Data(1,:);

%% Generalized speeds
yaw_rate = T.AV_Y;
pitch_rate = T.AV_P;
roll_rate = T.AV_R;
steer_rate = T.M_StrSys./0.2122;
px_rate = T.VxN_S2;
py_rate = T.VyN_S2;
pz_rate = T.VzN_S2;
euld = [yaw_rate,pitch_rate,roll_rate,steer_rate];
pd = [px_rate,py_rate];
qd = timeseries([deg2rad(euld),pd./3.6],t);

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
