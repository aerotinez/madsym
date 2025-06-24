close("all"); clear; clc;
setmadsympath();

%% Load validation data
speed = 130;
results_path = "G:\My Drive\BikeSimResults\BigSports\DLC";
speed_path = "\Vx" + num2str(speed) + "Kph\";
file_name = "bikesim_results_" + num2str(speed) + "kph.csv";
results = readtable(results_path + speed_path + file_name);

time = results.Time;

camber = deg2rad(results.Roll);
steer = deg2rad(results.Steer);
wz = deg2rad(results.AVz);
vy = results.VyW0_2./3.6;
wx = deg2rad(results.AVx);
ws = deg2rad(-results.M_StrSys./0.2212);
Mz = results.M_Str_In;

vx = results.VxW0_2./3.6;
vz = results.VzW0_2./3.6;
g = 9.80665;

wx2 = deg2rad(results.AVx_2);
wz2 = deg2rad(results.AVz_2);
vdy = (g.*results.Ay_WC2) - (wz2.*vx - wx2.*vz);

t = time;
ref = [camber,steer,wz,vy,wx,ws];
u = Mz;

%% Parameters
bs = bigSportsParameters;
s2m = @(x)cell2mat(struct2cell(x));
params = @(v)s2m(bikeSimToPrydeParameters(bs,v/3.6));

%% Model
sys = prydeMotorcycleLateralStateSpace(params(speed));

A = sys.E\sys.A;
B = sys.E\sys.B;

I = eye(size(A,1));

idxy = [1,3,4,5];

C = I(idxy,:);

D = 0;

nx = size(A,1);
nu = size(B,2);
ny = size(C,1);

sys = ss(A,B,C,D, ...
    'StateName',sys.StateName, ...
    'InputName',sys.InputName, ...
    'OutputName',sys.StateName(idxy));

%% Analysis

disp(rank(obsv(A,C)) == size(A,1));

ObsvCond = cond(obsv(A,C));
GrammCond = cond(gram(sys,'o'));

tab = table(ObsvCond,GrammCond);
disp(tab);

%% Balancing
[~,g,Tl,Tr] = balreal(sys,"noperm");

sysb = ss(Tl*A*Tr,Tl*B,C*Tr,0);

ObsvCond = cond(obsv(sysb));
GrammCond = cond(gram(sysb,'o'));

tab = table(ObsvCond,GrammCond);
disp(tab);

%% Design gains
L = lqr(sysb.A',sysb.C',sysb.C'*sysb.C,eye(ny))';

%% Construct observer
Ao = sysb.A - L*sysb.C;
Bo = [sysb.B,L];
Co = eye(nx);
Do = zeros(nx, size(Bo,2));
syso = ss(Ao,Bo,Co,Do);

%% Test observer
z = ref(:,idxy);

[~,~,est] = lsim(syso,[u,z],t);
y = (Tr*est.').';

%% Plot results

fig = figure;
tl = tiledlayout(3,2,"Parent",fig);

titles = [
    "Camber";
    "Steer";
    "Yaw rate (body-fixed)";
    "Lateral speed";
    "Camber rate (body-fixed)";
    "Steer rate"
    ];

for k = 1:6
    axe = nexttile(tl,k);
    hold(axe,'on');
    plot(t,ref(:,k),'LineWidth',1);
    plot(t,y(:,k),'LineWidth',1);
    hold(axe,'off');
    box(axe,'on');
    axis(axe,'tight');
    title(axe,titles(k));
    if ismember(k,[5,6])
        xlabel(axe,'Time (seconds)');
    end
end