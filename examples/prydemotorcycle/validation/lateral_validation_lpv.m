close("all"); clear; clc;
setmadsympath();

%% Parameters
bs = bigSportsParameters;
s2m = @(x)cell2mat(struct2cell(x));
disp(bikeSimToPrydeParameters(bs,30/3.6));
params = @(v)s2m(bikeSimToPrydeParameters(bs,v/3.6));

%% BikeSim results
Vx = [30,50,80,110,130];
sys = prydeMotorcycleLateralLPVStateSpace;
n2s = @num2str;
results_path = "G:\My Drive\BikeSimResults\BigSports\OpenLoop";

x_mes = cell(1,numel(Vx));
x_sys = cell(1,numel(Vx));

for k = 1:numel(Vx)
    speed_path = "\Vx" + n2s(Vx(k)) + "Kph\";
    file_name = "bikesim_results_" + n2s(Vx(k)) + "kph.csv";
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
    vy = results.VyW0_2./3.6;
    wx = results.AVx;
    ws = -results.M_StrSys./0.2212;

    vx = results.VxW0_2.';
    
    s = (180/pi).*[1,1,1,0,1,1] + [0,0,0,1,0,0];
    x_mes{k} = [camber,steer,wz,vy,wx,ws];
    
    Mz = results.M_Str_In;
    p = cell2mat(arrayfun(params,vx,'uniform',0)).';
    sf = (pi/180).*[1,1,1,0,1,1] + [0,0,0,1,0,0];
    IC = sf.*x_mes{k}(1,:);
    x_sys{k} = s.*lsim(sys,Mz,time,IC,p);
end

%% Plot results
w = 1366;
fig = figure("Position",[1,50,1280,1080]);
tl = tiledlayout(6,5,"Parent",fig,"TileSpacing","tight","Padding","compact");

titles = arrayfun(@(x)"Speed: " + x + "km/h",Vx);

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

ts = 1/120;

for k = 1:6*5
    [row,col] = ind2sub([5,6],k);
    axe = nexttile(tl,k);
    axe.LineWidth = 1;
    hold(axe,"on");
    ns = numel(x_sys{row}(:,col));
    time = linspace(0,(ns - 1)*ts,ns);
    plot(axe,time,x_mes{row}(:,col),"LineWidth",1);
    plot(axe,time,x_sys{row}(:,col),"LineWidth",1);
    hold(axe,"off");
    box(axe,"on");
    axis(axe,'tight');
    xlim(axe,[0,time(end)]);
    if row < 4 
        xlim(axe,[0,time(end)]);
    end
    if k < 6
        title(axe,titles(k),'FontSize',7)
    end
    if col == 4
        axe.YAxis.Exponent = -2;
    end
    if ismember(k,uind)
        ylabel(axe,units(j),'Interpreter','tex');
        j = j + 1;
    end
    if k <= 25
        xticks(axe,[]);
    end
    if k > 25
        xlabel(axe,"time (s)");
    end
end
ttl = "Lateral: Chicane results";
sgtitle(ttl);
leg = legend("ref (bikesim)","est (Pryde model)");
leg.Orientation = "horizontal";
leg.Layout.Tile = 'south';

%% Save figure
% dir = "C:\Users\marti\PhD\Thesis\MotorcycleDynamics\LinearModeling\Figures\";
% saveThesisFig(fig,dir + "chicane_results");