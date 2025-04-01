close("all"); clear; clc;
setmadsympath();

%% Parameters
bs = bigSportsParameters;
s2m = @(x)cell2mat(struct2cell(x));
disp(bikeSimToPrydeParameters(bs,30/3.6));
params = @(v)s2m(bikeSimToPrydeParameters(bs,v/3.6));

%% BikeSim results
vx = [30,50,80,110,130];
plant = @prydeMotorcycleLateralStateSpace;
n2s = @num2str;
results_path = "G:\My Drive\BikeSimResults\BigSports\OpenLoop";

x_mes = cell(1,numel(vx));
x_sys = cell(1,numel(vx));

for k = 1:numel(vx)
    speed_path = "\Vx" + n2s(vx(k)) + "Kph\";
    file_name = "bikesim_results_" + n2s(vx(k)) + "kph.csv";
    results = readtable(results_path + speed_path + file_name);
    
    time = results.Time;
    camber = results.Roll;
    steer = results.Steer;
    wz = results.AVz;
    vy = results.VyW0_2./3.6;
    wx = results.AVx;
    ws = -results.M_StrSys./0.2212;
    
    s = (180/pi).*[1,1,1,0,1,1] + [0,0,0,1,0,0];
    x_mes{k} = [camber,steer,wz,vy,wx,ws];
    
    Mz = results.M_Str_In;
    p = params(vx(k));
    sys = plant(p);
    sf = (pi/180).*[1,1,1,0,1,1] + [0,0,0,1,0,0];
    IC = sf.*x_mes{k}(1,:);
    x_sys{k} = s.*lsim(sys,Mz,time,IC);
end

%% Plot results
fig = figure("Position",[570,100,1280,720]);
tl = tiledlayout(6,5,"Parent",fig);

titles = arrayfun(@(x)"Speed: " + x + "km/h",vx);

units = [
    "\gamma (\circ)";
    "\delta (\circ)";
    "\omega_z (\circ/s)";
    "v_y (m/s)";
    "\omega_x (\circ/s)";
    "\omega_\delta (\circ/s)"
    ];

uind = (0:5:25) + 1;
j = 1;

ts = 1/60;

for k = 1:6*5
    [row,col] = ind2sub([5,6],k);
    axe = nexttile(tl,k);
    hold(axe,"on");
    ns = numel(x_sys{row}(:,col));
    time = linspace(0,(ns - 1)*ts,ns);
    plot(axe,time,x_mes{row}(:,col),"LineWidth",1.5);
    plot(axe,time,x_sys{row}(:,col),"LineWidth",1.5);
    hold(axe,"off");
    xlim(axe,[0,time(end)]);
    box(axe,"on");
    axis(axe,'tight');
    if k < 6
        title(axe,titles(k),'FontSize',14)
    end
    if ismember(k,uind)
        ylabel(axe,units(j),'Interpreter','tex','FontSize',14);
        j = j + 1;
    end
    if k <= 25
        xticks(axe,[]);
    end
    if k > 25
        xlabel(axe,"time (s)","FontSize",14);
    end
end
ttl = "Small sinusoidal pertubation results (open loop) ";
sgtitle(ttl,'FontSize',22);
leg = legend("ref (bikesim)","est (Pryde model)",'FontSize',14);
leg.Orientation = "horizontal";
leg.Layout.Tile = 'south';

%% Save figure
% dir = 'C:\Users\marti\PhD\Thesis\MotorcycleDynamics\Validation\Figures\';
% saveas(fig,string(dir) + "open_loop_results.eps",'epsc');