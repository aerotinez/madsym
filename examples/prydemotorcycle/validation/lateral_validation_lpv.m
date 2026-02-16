close('all'); clear; clc;
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
scen = "Chicane";
results_path = "G:\My Drive\BikeSimResults\BigSports\" + strrep(scen,' ','');

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
fig = figure("Position",[1,50,960,540]);
% fig = figure("Position",[100,100,1280,]);
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

rmse_results = zeros(5,numel(Vx));
percentage_rmse_results = zeros(5,numel(Vx));

for k = 1:6*5
    [row,col] = ind2sub([5,6],k);
    axe = nexttile(tl,k);
    axe.LineWidth = 1;
    hold(axe,"on");
    ns = numel(x_sys{row}(:,col));
    time = linspace(0,(ns - 1)*ts,ns);
    plot(axe,time,x_mes{row}(:,col),"LineWidth",1.5);
    plot(axe,time,x_sys{row}(:,col),"LineWidth",1.5);
    hold(axe,"off");
    box(axe,"on");
    axis(axe,'tight');
    xlim(axe,[0,time(end)]);
    if row < 4 
        xlim(axe,[0,time(end)]);
    end
    if k < 6
        title(axe,titles(k),'FontSize',12)
    end
    if col == 4
        axe.YAxis.Exponent = -2;
    end
    if ismember(k,uind)
        ylabel(axe,units(j),'Interpreter','tex','FontSize',12);
        j = j + 1;
    end
    if k <= 25
        xticks(axe,[]);
    end
    if k > 25
        xlabel(axe,"time (s)",'FontSize',12);
    end
    rmse_results(col,row) = rmse(x_sys{row}(:,col),x_mes{row}(:,col))';
    percentage_rmse_results(col,row) = 100*(rmse(x_sys{row}(:,col),x_mes{row}(:,col))./(max(x_mes{row}(:,col)) - min(x_mes{row}(:,col))))';
end
ttl = "Lateral: " + scen + " results";
sgtitle(ttl,'FontSize',12);
leg = legend("ref (bikesim)","est (Pryde model)",'FontSize',12);
leg.Orientation = "horizontal";
leg.Layout.Tile = 'south';

%% Save figure
dir = "C:\Users\marti\PhD\Thesis\MotorcycleDynamics\LinearModeling\Figures\";
saveas(fig,dir + strrep(lower(scen),' ','_') + "_results",'epsc');

%% Error table

txt = [
    "\\begin{tabularx}{\\textwidth}{|C|C||*{5}{C|}}\n";
    "\\hline\n";
    "\\multirow{2}{*}{State} &\n";
    "\\multirow{2}{*}{Units} &\n";
    "\\multicolumn{5}{c|}{\n";
    "Speed ($\\unit[per-mode=symbol]{\\kilo\\meter\\per\\hour}$)\n";
    "} \\\\\n";
    "\\cline{3-7}\n";
    "& & 30 & 50 & 80 & 110 & 130 \\\\\n";
    "\\hline\\hline\n"
    ];

states = [
    "$\\gamma$";
    "$\\delta$";
    "$\\omega_{z}$";
    "$v_{y}$";
    "$\\omega_{x}$";
    "$\\omega_{\\delta}$"
    ];

units = [
    "\\degree";
    "\\degree";
    "\\degree\\per\\second";
    "\\meter\\per\\second";
    "\\degree\\per\\second";
    "\\degree\\per\\second"
    ];

dir_name = "C:\Users\marti\PhD\Thesis\MotorcycleDynamics\LinearModeling\";

fid = fopen(dir_name + strrep(lower(scen),' ','_') + "_rmse_results.tex","w");
arrayfun(@(s)fprintf(fid,s),txt);
ftxt = @(x)"& " + num2str(x,'%.4f');

for k = 1:5
    txt = states(k) + " & $\\unit[per-mode=symbol]{" + units(k) + "}$ ";
    txt = txt + strjoin(arrayfun(ftxt,rmse_results(k,:)));
    txt = txt + "\\\\\n\\hline\n";
    fprintf(fid,txt);
end

fprintf(fid,"\\end{tabularx}");
fclose(fid);

percentage_rmse_results(isnan(percentage_rmse_results)) = 0;
percentage_rmse_results = mean(percentage_rmse_results,2)';

fid = fopen(dir_name + strrep(lower(scen),' ','_') + "_prmse_results.tex","w");
ftxt = @(x)string(num2str(x,'%.4f'));

txt = [
    "\\begin{tabularx}{\\textwidth}{|C|C|C|C|C|C|}\n";
    "\\hline\n";
    strjoin(states'," & ") + "\\\\\n";
    "\\hline\\hline\n";
    strjoin(arrayfun(ftxt,percentage_rmse_results)," & ") + "\\\\\n";
    "\\hline\n";
    "\\end{tabularx}\n"
    ];

arrayfun(@(s)fprintf(fid,s),txt);
fclose(fid);