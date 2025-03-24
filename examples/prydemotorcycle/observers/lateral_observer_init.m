close("all"); clear; clc;
setmadsympath();

%% Parameters
bs = bigSportsParameters;
s2m = @(x)cell2mat(struct2cell(x));
disp(bikeSimToPrydeParameters(bs,30/3.6));
params = @(v)s2m(bikeSimToPrydeParameters(bs,v/3.6));
ts = 1/60;

%% BikeSim results
vx = [30,50,80,110,130];
plant = @prydeMotorcycleLateralStateSpace;
n2s = @num2str;
results_path = "G:\My Drive\BikeSimResults\BigSports\DLC";

k = 4;

speed_path = "\Vx" + n2s(vx(k)) + "Kph\";
file_name = "bikesim_results_" + n2s(vx(k)) + "kph.csv";
results = readtable(results_path + speed_path + file_name);

time = results.Time;

camber = timeseries(deg2rad(results.Roll),time);
steer = timeseries(deg2rad(results.Steer),time);
wz = timeseries(deg2rad(results.AVz),time);
vy = timeseries(results.VyW0_2./3.6,time);
wx = timeseries(deg2rad(results.AVx),time);
ws = timeseries(deg2rad(-results.M_StrSys./0.2212),time);

x = [camber,steer,wz,vy,wx,ws];
x0 = arrayfun(@(x)x.Data(1),x);

Mz = timeseries(results.M_Str_In,time);
p = params(vx(k));
sys = plant(p);

A = sys.A;
B = sys.B(:,2);

C = eye(6);

L = designObserverGains(A,C);

function L = designObserverGains(A,C)
    nx = size(A,1);
    ny = size(C,1);

    yalmip('clear');
    P = sdpvar(nx,nx,'symmetric');
    R = sdpvar(ny,nx,'full');
    I = eye(nx);

    g = 1E-03;
    t = 1E-09;

    F = [
        P >= eps*I;
        [
        A'*P + P*A - R'*C - C'*R + t*g*g*I, P;
        P, -t*I
        ] <= -eps*eye(2*nx)
        ];

    opts = sdpsettings('solver','sdpt3');
    optimize(F,[],opts);

    L = value(P)\value(R)';
end