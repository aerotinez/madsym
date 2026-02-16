close("all"); clear; clc;
setmadsympath();

%% Parameters
bs = bigSportsParameters;
s2m = @(x)cell2mat(struct2cell(x));
disp(bikeSimToPrydeParameters(bs,30/3.6));
params = @(v)s2m(bikeSimToPrydeParameters(bs,v/3.6));

%% BikeSim results
SPEED = [30,50,80,110,130];
plant = @prydeMotorcycleNoSlipStateSpace;
n2s = @num2str;
results_path = "G:\My Drive\BikeSimResults\BigSports\OpenLoop";

x_mes = cell(1,numel(SPEED));
x_sys = cell(1,numel(SPEED));

for k = 1:numel(SPEED)
    speed_path = "\Vx" + n2s(SPEED(k)) + "Kph\";
    file_name = "bikesim_results_" + n2s(SPEED(k)) + "kph.csv";
    results = readtable(results_path + speed_path + file_name);
    
    time = results.Time;

    yaw = deg2rad(results.Yaw);
    pitch = deg2rad(results.Pitch);
    roll = deg2rad(results.Roll_E);

    R = angle2dcm(yaw,pitch,roll);
    [yaw,camber,pitch] = dcm2angle(R,'ZXY');

    camber = rad2deg(camber);
    steer = results.Steer;
    wz = results.AVz;
    vx = results.VxW0_2/3.6;
    wx = results.AVx;
    
    s = (180/pi).*[1,1,1,0,1] + [0,0,0,1,0];
    x_mes{k} = [camber,steer,wz,vx,wx];
    
    My = results.My_DR_2;
    Mbr = -results.My_Bk_2;
    Mbf = -results.My_Bk_1;
    Mz = results.M_Str_In;

    p = params(SPEED(k));
    sys = plant(p);

    sf = (pi/180).*[1,1,1,0,1] + [0,0,0,1,0];
    IC = sf.*x_mes{k}(1,:);

    F = [My,Mbr,Mbf,Mz];
    [~,~,x] = lsim(sys,F,time,IC,params);
    x_sys{k} = s.*x;
end

%% Plot results
fig = figure("Position",[100,50,1280,720]);
tl = tiledlayout(5,5,"Parent",fig);

titles = arrayfun(@(x)"Speed: " + x + "km/h",SPEED);

units = [
    "\gamma (\circ)";
    "\delta (\circ)";
    "\omega_z (\circ/s)";
    "v_x (m/s)";
    "\omega_x (\circ/s)"
    ];

uind = (0:5:25) + 1;
j = 1;

ts = 1/120;

for k = 1:25
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
        title(axe,titles(k),'FontSize',12)
    end
    if ismember(k,uind)
        ylabel(axe,units(j),'Interpreter','tex','FontSize',12);
        j = j + 1;
    end
    if k <= 25
        xticks(axe,[]);
    end
    if k > 25
        xlabel(axe,"time (s)","FontSize",12);
    end
end
ttl = "Small sinusoidal pertubation results (open loop) ";
sgtitle(ttl,'FontSize',22);
leg = legend("ref (bikesim)","est (Pryde model)",'FontSize',12);
leg.Orientation = "horizontal";
leg.Layout.Tile = 'south';

%% Save figure
% dir = 'C:\Users\marti\PhD\Thesis\MotorcycleDynamics\Validation\Figures\';
% saveas(fig,string(dir) + "open_loop_results.eps",'epsc');