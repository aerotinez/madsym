close("all"); clc;
setmadsympath();

%% Load dataset
vx = 50;
n2s = @num2str;
results_path = "G:\My Drive\BikeSimResults\BigSports\OpenLoop";
speed_path = "\Vx" + n2s(vx) + "Kph\";
file_name = "bikesim_results_" + n2s(vx) + "kph.csv";
results = readtable(results_path + speed_path + file_name);
% load("G:\My Drive\BikeSimResults\BigSports\Slalom50Kph\results_slalom_50kph.mat");
% results = readtable("G:\My Drive\BikeSimResults\BigSports\ICRA25\Vx50Kph\bikesim_results_50kph.csv");


%% Signal data
time = results.Time;
camber = timeseries(results.Roll,time);
steer = timeseries(results.Steer,time);
wz = timeseries(results.AVz,time);
vy = timeseries(results.VyW0_2./3.6,time);
wx = timeseries(results.AVx,time);
ws = timeseries(-results.M_StrSys./0.2212,time);

Mz = timeseries(results.M_Str_In,time);

%% Initial condition
x = [camber.Data,steer.Data,wz.Data,vy.Data,wx.Data,ws.Data];
s = (pi/180).*[1,1,1,0,1,1] + [0,0,0,1,0,0];
IC = s.*x(1,:);

%% Initial parameters
bs = bigSportsParameters;
s2m = @(x)cell2mat(struct2cell(x));
params = s2m(bikeSimToPrydeParameters(bs,vx/3.6));
kmzaf = params(7);
kmzar = params(8);
kmzlf = params(9);
kmzlr = params(10);
kfyaf = params(12);
kfyar = params(13);
kfylf = params(14);
kfylr = params(15);