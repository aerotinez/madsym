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
plant = @prydeMotorcycleUILateralStateSpace;
n2s = @num2str;
results_path = "G:\My Drive\BikeSimResults\BigSports\Chicane";

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
E = sys.B(:,3:end);

C = [
    1,0,0,0,0,0;
    0,0,1,0,0,0;
    0,0,0,1,0,0;
    0,0,0,0,1,0
    ];

L = designObserverGains(A,C,E);

disp(rank(C*E) == rank(E));

% H = E*(((C*E).'*C*E)\(C*E).');
% I = eye(6);
% T = I - H*C;
% F = A - K1*C;
% K2 = F*H;
% K = K1 + K2;
% 
% A1 = T*A;
% disp(isDetectable(A1,C));


function L = designObserverGains(A,C,E)
    nx = size(A,1);
    ny = size(C,1);
    I = eye(nx);

    yalmip('clear');
    P = sdpvar(nx,nx,'symmetric');
    R = sdpvar(nx,ny,'full');
    g = sdpvar(1,1);

    ep = 1E-06;
    a = 20;

    F = [
        P >= ep*I;
        R*C*E - P*E <= ep*I;
        g >= ep;
        [
        A'*P + P*A - C'*R' - R*C  + 2*a*P, -R;
        -R', -g*eye(ny)
        ] <= -ep*eye(nx + ny)
        ];

    opts = sdpsettings('solver','sdpt3');
    optimize(F,g,opts);

    L = value(P)\value(R);
end

function res = isDetectable(A,C)
    for lambda = eig(A)'
        if real(lambda) >= 0
            if rank([A - lambda*eye(size(A)); C]) < size(A,1)
                res = false;
                return;
            end
        end
    end
    res = true;
end