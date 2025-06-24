close all; clear; clc;
setmadsympath();

%% Load validation data
speed = 130;
results_path = "G:\My Drive\BikeSimResults\BigSports\Chicane";
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

ns = 2000;
t = linspace(time(1), time(end), ns)';
ref = interp1(time, [camber, steer, wz, vy, wx, ws], t);
u = interp1(time, Mz, t);

%% Parameters
bs = bigSportsParameters;
s2m = @(x)cell2mat(struct2cell(x));
disp(bikeSimToPrydeParameters(bs,30/3.6));
params = @(v)s2m(bikeSimToPrydeParameters(bs,v/3.6));

%% Model
sys_struct = prydeMotorcycleLateralStateSpace(params(speed));
A = sys_struct.E \ sys_struct.A;
B = sys_struct.E \ sys_struct.B;

C = [
    1,0,0,0,0,0;
    0,0,1,0,0,0;
    0,0,0,1,0,0;
    0,0,0,0,1,0
];

D = zeros(size(C,1), size(B,2));
sys = ss(A, B, C, D);

%% Define noise covariances
nx = size(A,1);  % State dimension
nu = size(B,2);
ny = size(C,1);  % Output dimension

Q = 1e-4 * eye(nu);  % Process noise covariance
R = 1e-2 * eye(ny);  % Measurement noise covariance

%% Design Kalman estimator using built-in function
[kest, L, P] = kalman(ss(A, [B, eye(nx)], C, 0), Q, R);

%% Simulate
y_meas = ref(:, [1,3,4,5]);  % Measured outputs
xhat = lsim(kest, [u, y_meas], t, ref(1,:));  % Simulate estimator

%% Plot results
titles = [
    "Camber";
    "Steer";
    "Yaw rate (body-fixed)";
    "Lateral speed";
    "Camber rate (body-fixed)";
    "Steer rate"
];

fig = figure;
tl = tiledlayout(3,2,"Parent",fig);

for i = 1:6
    axe = nexttile(tl,i);
    hold(axe,'on');
    plot(t, ref(:,i), 'LineWidth', 1.2);
    plot(t, xhat(:,i), 'LineWidth', 1);
    hold(axe,'off');
    title(axe, titles(i));
    legend(axe, 'Ground Truth', 'KF Estimate');
    box(axe, 'on'); axis(axe, 'tight');
    if ismember(i, [5,6])
        xlabel('Time (s)');
    end
end
