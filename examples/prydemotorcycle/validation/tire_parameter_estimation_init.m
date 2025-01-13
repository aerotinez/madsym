close("all"); clc;

%% Load dataset
vx = 50;
n2s = @num2str;
results_path = "G:\My Drive\BikeSimResults\BigSports\OpenLoop";
speed_path = "\Vx" + n2s(vx) + "Kph\";
file_name = "bikesim_results_" + n2s(vx) + "kph.csv";
results = readtable(results_path + speed_path + file_name);

%% Signal data
time = results.Time;
camber = timeseries(results.Roll,time);
steer = timeseries(results.Steer,time);
wz = timeseries(results.AVz,time);
vy = timeseries(results.VyW0_2./3.6,time);
wx = timeseries(results.AVx,time);
ws = timeseries(-results.M_StrSys./0.2212,time);

Mz = timeseries(results.M_Str_In,time);

%% Initial parameters
bs = bigSportsParameters;
s2m = @(x)cell2mat(struct2cell(x));
params = s2m(bikeSimToPrydeParameters(bs,vx/3.6));
% kmzaf = params(7);
% kmzar = params(8);
% kmzlf = params(9);
% kmzlr = params(10);
% kfyaf = params(12);
% kfyar = params(13);
% kfylf = params(14);
% kfylr = params(15);
kfyaf = 27.858;
kfyar = 9.5992;
kfylf = 0.0086564;
kfylr = 0.03315;
kmzaf = 0.68051;
kmzar = 4.9337;
kmzlf = 0.062821;
kmzlr = 0.064109;