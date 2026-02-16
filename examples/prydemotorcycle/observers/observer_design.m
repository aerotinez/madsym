close("all"); clear; clc;
setmadsympath();

%% Load validation data
speed = 130;
results_path = "G:\My Drive\BikeSimResults\BigSports\OpenLoop";
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

ns = 10*numel(time);
t = interp1(linspace(0,1,numel(time)),time,linspace(0,1,ns));
ref = interp1(time,[camber,steer,wz,vy,wx,ws],t,"makima");
u = interp1(time,Mz,t,"makima")';

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

%% Design observer
y_meas = ref(:,idxy);

% Observability check
if rank(obsv(A,C)) < nx
    error('System (A,C) is not observable: rank=%d < n=%d',rank(obsv(A,C)),nx);
end

L = place(A',C',7*pole(sys)')';   % observer gain

% Observer dynamics:  xhat_dot = (A - L*C) xhat + [B  L] * [u; y_meas]
Aobs = A - L*C;
Bobs = [B, L];
Cobs = eye(nx);
Dobs = zeros(nx, nu + ny);
obs  = ss(Aobs, Bobs, Cobs, Dobs);

Uobs  = [u,y_meas];
xhat0 = zeros(nx,1);
y = lsim(obs, Uobs, t, xhat0);

%% Plot results

idx = [1:3,5,6];
ref(:,idx) = rad2deg(ref(:,idx));
y(:,idx) = rad2deg(y(:,idx));

fig = figure("Position",[100,100,840,300]);
tl = tiledlayout(2,3,"Parent",fig,"TileSpacing","compact","Padding","compact");

titles = [
    "Camber";
    "Steer";
    "Yaw rate (body-fixed)";
    "Lateral speed";
    "Camber rate (body-fixed)";
    "Steer rate"
    ];

labels = [
    "angle (\circ)";
    "angle (\circ)";
    "speed (\circ/s)";
    "speed (m/s)";
    "speed (\circ/s)";
    "speed (\circ/s)"
    ];

for k = 1:6
    axe = nexttile(tl,k);
    hold(axe,'on');
    plot(t,ref(:,k),'LineWidth',1);
    plot(t,y(:,k),'LineWidth',1);
    hold(axe,'off');
    box(axe,'on');
    axis(axe,'tight');
    title(axe,titles(k),"FontSize",12);
    ylabel(axe,labels(k),"FontSize",12);
    if ismember(k,[4,5,6])
        xlabel(axe,'Time (seconds)',"FontSize",12);
    end
end