close("all"); clear; clc;
setmadsympath();

%% Parameters
bs = bigSportsParameters;
% bs.SteeringHead.Damping = 0.5;
s2m = @(x)cell2mat(struct2cell(x));
disp(bikeSimToPrydeV2Parameters(bs,30/3.6));
params = @(v)s2m(bikeSimToPrydeV2Parameters(bs,v/3.6));

%% Model
nv = 500;
vmin = 1;
vmax = 300;
vx = linspace(vmin,vmax,nv);
plant = @prydeMotoLateralStateSpace;
f = @(v)plant(params(v));
sys = arrayfun(f,vx.',"uniform",0);

%% Poles
poles = cell2mat(cellfun(@pole,sys,"uniform",0).');

%% Natural frequencies
m = matlabColors();

%% Nyquist plot
fig = figure('Position',[100,100,640,360]);
axe = axes(fig);
cm = parula;
c = interp1(linspace(0,1,size(cm,1)).',cm,linspace(0,1,numel(vx)).');
h = scatter(axe,real(poles),imag(poles),20,c,"filled");

xticks(axe,[]);
yticks(axe,[]);
title(axe,"Lateral: Nyquist response","FontSize",12);
xlabel(axe,"Re (1/s)","FontSize",12);
ylabel(axe,"Im (rad/s)","FontSize",12);
box(axe,"on");
% xlim(axe,[-450,6])
sgrid(axe);
cb = colorbar(axe);
f = @(x)string((vmax - vmin)*double(string(x)) + vmin);
cb.TickLabels = cellfun(f,cb.TickLabels,'uniform',0);
cb.Label.String = "Speed (km/h)";
cb.Label.FontSize = 12;

% dir = "C:\Users\marti\PhD\Thesis\MotorcycleDynamics\LinearModeling\Figures\";
% saveThesisFig(fig,dir + "lateral_nyquist");