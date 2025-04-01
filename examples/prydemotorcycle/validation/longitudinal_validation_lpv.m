close("all"); clear; clc;
setmadsympath();

%% Parameters
bs = bigSportsParameters;
s2m = @(x)cell2mat(struct2cell(x));
disp(bikeSimToPrydeParameters(bs,30/3.6));
params = @(t,x,u)s2m(bikeSimToPrydeParameters(bs,x(1)));

%% BikeSim results
Vx = [30,50,80,110,130];
sys = prydeMotorcycleLongitudinalLPVStateSpace;
n2s = @num2str;
results_path = "G:\My Drive\BikeSimResults\BigSports\DLC";

x_mes = cell(1,numel(Vx));
x_sys = cell(1,numel(Vx));

for k = 1:numel(Vx)
    speed_path = "\Vx" + n2s(Vx(k)) + "Kph\";
    file_name = "bikesim_results_" + n2s(Vx(k)) + "kph.csv";
    results = readtable(results_path + speed_path + file_name);
    
    time = results.Time;
    vx = results.VxW0_2./3.6;
    wr = results.AVy_W2*6;
    wf = results.AVy_W1*6;
    
    s = (180/pi).*[0,1,1] + [1,0,0];
    x_mes{k} = [vx,wr,wf];
    
    My = results.My_DR_2;
    sf = (pi/180).*[0,1,1] + [1,0,0];
    IC = sf.*x_mes{k}(1,:);
    x_sys{k} = s.*lsim(sys,My,time,IC,params);
end

%% Plot results
fig = figure("Position",[570,100,1280,720]);
tl = tiledlayout(3,5,"Parent",fig);

titles = arrayfun(@(x)"Speed: " + x + "km/h",Vx);

units = [
    "v_x (m/s)";
    "\omega_\theta_r (\circ/s)";
    "\omega_\theta_f (\circ/s)"
    ];

uind = (0:5:25) + 1;
j = 1;

ts = 1/60;

for k = 1:3*5
    [row,col] = ind2sub([5,3],k);
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
    if k <= 12
        xticks(axe,[]);
    end
    if k > 12
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