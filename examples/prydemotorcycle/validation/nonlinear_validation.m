close("all"); clear; clc;
setmadsympath();

%% BikeSim results
speed = 130;
results_path = "G:\My Drive\BikeSimResults\BigSports\Chicane";
speed_path = "\Vx" + speed + "Kph\";
file_name = "bikesim_results_" + speed + "kph.csv";
ref = readtable(results_path + speed_path + file_name);

time = ref.Time;

%% Generalized coordinates
R = angle2dcm(deg2rad(ref.Yaw),deg2rad(ref.Pitch),deg2rad(ref.Roll_E));
[yaw_r,camber_r,pitch] = dcm2angle(R,'ZXY');

pxr = ref.X_WC2;
pyr = ref.Y_WC2;
pitch_r = deg2rad(ref.Rot_W2);
steer = deg2rad(ref.Steer);
pitch_f = deg2rad(ref.Rot_W1);

R = angle2dcm(deg2rad(ref.YawE_1),deg2rad(ref.Pitch_1),deg2rad(ref.RollE_1));
[yaw_f,camber_f,~] = dcm2angle(R,'ZXY');

pfx = ref.X_WC1;
pfy = ref.Y_WC1;

q = [yaw_r,pxr,pyr,camber_r,pitch_r,steer,pitch_f,pitch,yaw_f,pfx,pfy,camber_f];

%% Quasi-velocities
wrz = deg2rad(ref.AVz);
vrx = ref.VxW0_2./3.6;
vry = ref.VyW0_2./3.6;
wrx = deg2rad(ref.AVx);
wr = (2*pi/60)*ref.AVy_Wf2;
ws = -ref.M_StrSys./0.2212;
wf = (2*pi/60)*ref.AVy_Wf1;
wry = deg2rad(ref.AVy);
wfz = deg2rad(ref.AVz_1);
vfx = ref.VxW0_1./3.6;
vfy = ref.VyW0_1./3.6;
wfx = deg2rad(ref.AVx_1);

u = [wrz,vrx,vry,wrx,wr,ws,wf,wry,wfz,vfx,vfy,wfx];

%% States
x = [q,u];
x0 = x(1,:);
x0(22) = vrx(1);

%% Control inputs
My = ref.My_DR_2;
Mbr = -ref.My_Bk_2;
Mbf = -ref.My_Bk_1;
Mz = ref.M_Str_In;

u = [My,Mbr,Mbf,Mz];

%% 

bs = bigSportsParameters;
s2m = @(x)cell2mat(struct2cell(x));
disp(bikeSimToPrydeParameters(bs,30/3.6));
params = @(v)s2m(bikeSimToPrydeParameters(bs,v/3.6));

p = params(speed);
p(16) = [];

fu = @(t)interp1(time,u,t,"cubic").';

f = @(t,y)prydeMotorcycleDAEForcingVector(y(:),fu(t),p);
M = @(t,y)prydeMotorcycleDAEMassMatrix(y(:),p);
opts = odeset('Mass',M,'MStateDependence','strong','MaxStep',1/120);

[t,x_est] = ode15s(f,time.',x(1,:),opts);