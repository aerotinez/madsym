close("all"); clear; clc;
setmadsympath();

%% Parameters
bs = bigSportsParameters;
s2m = @(x)cell2mat(struct2cell(x));
pars = bikeSimToSharpParameters(bs,30/3.6);
disp(pars);
params = @(v)s2m(bikeSimToSharpParameters(bs,v/3.6));

%% BikeSim results
vx = [30,50,80,110,130];
plant = @sharpMotorcycleStateSpace;
n2s = @num2str;

scen = "Chicane";
results_path = "G:\My Drive\BikeSimResults\BigSports\" + scen;

x_mes = cell(1,numel(vx));
x_sys = cell(1,numel(vx));

l = (pars.a -pars.an)/cos(pars.caster);

for k = 1:numel(vx)
    speed_path = "\Vx" + n2s(vx(k)) + "Kph\";
    file_name = "bikesim_results_" + n2s(vx(k)) + "kph.csv";
    results = readtable(results_path + speed_path + file_name);
    p = params(vx(k));

    w = deg2rad(table2array(results(:,["AVx","AVy","AVz"])));
    v = table2array(results(:,["Vxo","Vyo","Vzo"]))/3.6;

    va = cross(w,[-l,0,0].*ones(size(w))) + v;
    
    time = results.Time;
    camber = -results.Roll;
    steer = results.Steer;
    wz = results.AVz;
    vy = va(:,2);
    wx = -results.AVx;
    ws = -results.M_StrSys./0.2212;
    Yf = results.Fy_1;
    Yr = results.Fy_2;
    
    s = (180/pi).*[1,1,1,0,1,1,0,0] + [0,0,0,1,0,0,1,1];
    x_mes{k} = [camber,steer,wz,vy,wx,ws,Yr,Yf];
    
    Mz = results.M_Str_In;
    sys = plant(p);
    sf = (pi/180).*[1,1,1,0,1,1,0,0] + [0,0,0,1,0,0,1,1];
    IC = sf.*x_mes{k}(1,:);
    x_sys{k} = s.*lsim(sys,Mz,time,IC);
end

%% Plot results
fig = figure("Position",[-1080,1,1080,840]);

tl = tiledlayout(8,5, ...
    "Parent",fig, ...
    "Padding","compact", ...
    "TileSpacing","tight");

titles = arrayfun(@(x)"Speed: " + x + "km/h",vx);

units = [
    "\gamma (\circ)";
    "\delta (\circ)";
    "\omega_z (\circ/s)";
    "v_y (m/s)";
    "\omega_x (\circ/s)";
    "\omega_\delta (\circ/s)";
    "Y_r (N)";
    "Y_f (N)"
    ];

uind = (0:5:35) + 1;
j = 1;

ts = 1/60;

for k = 1:8*5
    [row,col] = ind2sub([5,8],k);
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
    if k <= 35
        xticks(axe,[]);
    end
    if k > 35
        xlabel(axe,"time (s)","FontSize",12);
    end

    % RMSE for current state/speed pair
    rmse_results(col,row) = rmse(x_sys{row}(:,col),x_mes{row}(:,col));
end

dict = dictionary(["OpenLoop","DLC","Chicane"],["Open loop","Double Lane-Change","Chicane"]);

ttl = "Sharp model (" + dict(scen) + ")";
sgtitle(ttl,'FontSize',12);

leg = legend("ref","est",'FontSize',12);
leg.Orientation = "horizontal";
leg.Layout.Tile = 'south';

%% Save figure
% 
% dict = dictionary(["OpenLoop","DLC","Chicane"],["open_loop","dlc","chicane"]);
% 
% dir = "C:\Users\marti\PhD\Articles\VSD26\Figures\";
% saveas(fig,dir + dict(scen) + "_results_sharp.eps",'epsc');
% 
% %% RMSE table
% states = [
%     "$\gamma$";
%     "$\delta$";
%     "$\omega_{z}$";
%     "$v_{y}$";
%     "$\omega_{x}$";
%     "$\omega_{\delta}$";
%     "$Y_{r}$";
%     "$Y_{f}$"
%     ];
% 
% latex_units = [
%     "\degree";
%     "\degree";
%     "\degree\per\second";
%     "\meter\per\second";
%     "\degree\per\second";
%     "\degree\per\second";
%     "\newton";
%     "\newton"
%     ];
% 
% table_dir = "C:\Users\marti\PhD\Articles\VSD26\";
% 
% fid = fopen(table_dir + dict(scen) + "_rmse_results_sharp.tex","w");
% 
% fprintf(fid,"\\begin{tabularx}{\\textwidth}{|C|C||*{5}{C|}}\n");
% fprintf(fid,"\\hline\n");
% fprintf(fid,"\\multirow{2}{*}{State} &\n");
% fprintf(fid,"\\multirow{2}{*}{Units} &\n");
% fprintf(fid,"\\multicolumn{5}{c|}{Speed ($\\unit[per-mode=symbol]{\\kilo\\meter\\per\\hour}$)} \\\\\n");
% fprintf(fid,"\\cline{3-7}\n");
% fprintf(fid,"& & 30 & 50 & 80 & 110 & 130 \\\\\n");
% fprintf(fid,"\\hline\\hline\n");
% 
% for k = 1:8
%     fprintf(fid,"%s & $\\unit[per-mode=symbol]{%s}$ ", states(k), latex_units(k));
%     fprintf(fid,"& %.4f & %.4f & %.4f & %.4f & %.4f ", rmse_results(k,:));
%     fprintf(fid,"\\\\\n\\hline\n");
% end
% 
% fprintf(fid,"\\end{tabularx}\n");
% fclose(fid);