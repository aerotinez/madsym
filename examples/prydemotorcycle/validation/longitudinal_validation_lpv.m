close("all"); clear; clc;
setmadsympath();

%% Parameters
bs = bigSportsParameters;
s2m = @(x)cell2mat(struct2cell(x));
disp(bikeSimToPrydeParameters(bs,30/3.6));
params = @(v)s2m(bikeSimToPrydeParameters(bs,v));

%% BikeSim results
Vx = [30,50,80,110,130];
sys = prydeMotorcycleLongitudinalLPVStateSpace;
n2s = @num2str;
results_path = "G:\My Drive\BikeSimResults\BigSports\Braking";

x_mes = cell(1,numel(Vx));
x_sys = cell(1,numel(Vx));
t = cell(1,numel(Vx));

for k = 1:numel(Vx)
    speed_path = "\Vx" + n2s(Vx(k)) + "Kph\";
    file_name = "bikesim_results_" + n2s(Vx(k)) + "kph.csv";
    results = readtable(results_path + speed_path + file_name);
    results = results(100:end,:);

    time = results.Time;
    vx = results.VxW0_2./3.6;
    wr = results.AVy_W2/60;
    wf = results.AVy_W1/60;
    
    s = (1/(2*pi)).*[0,1,1] + [1,0,0];
    x_mes{k} = [vx,wr,wf];
    
    My = results.My_DR_2;
    Mbr = -results.My_Bk_2;
    Mbf = -results.My_Bk_1;
    p = @(t,x,u)params(x(1));
    sf = (2*pi).*[0,1,1] + [1,0,0];
    IC = sf.*x_mes{k}(1,:);
    [~,~,x] = lsim(sys,[My,Mbr,Mbf],time,IC,p);
    x_sys{k} = s.*x;
    t{k} = time;
end

%% Plot results
fig = figure("Position",[570,100,720,320]);
tl = tiledlayout(3,5,"Parent",fig,"TileSpacing","compact","Padding","tight");
% tl.TileSpacing = "loose";

titles = arrayfun(@(x)"Speed: " + x + "km/h",Vx);

units = [
    "v_x (m/s)";
    "\omega_\theta_r (Hz)";
    "\omega_\theta_f (Hz)"
    ];

uind = (0:5:25) + 1;
j = 1;

ts = 1/60;

rmse_results = zeros(3,numel(Vx));
percentage_rmse_results = zeros(3,numel(Vx));

for k = 1:3*5
    [row,col] = ind2sub([5,3],k);
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
    xlim(axe,[time(1),time(end)]);
    % if col > 1
    %     axe.YAxis.Exponent = 3;
    % end
    if k < 6
        title(axe,titles(k),'FontSize',12);
    end
    if ismember(k,uind)
        ylabel(axe,units(j),'Interpreter','tex','FontSize',12);
        j = j + 1;
    end
    if k <= 10
        xticks(axe,[]);
    end
    if k > 10
        xlabel(axe,"time (s)",'FontSize',12);
    end
    rmse_results(col,row) = rmse(x_sys{row}(:,col),x_mes{row}(:,col))';
    percentage_rmse_results(col,row) = 100*(rmse(x_sys{row}(:,col),x_mes{row}(:,col))./(max(x_mes{row}(:,col)) - min(x_mes{row}(:,col))))';
end
ttl = "Longitudinal: throttle/braking results ";
sgtitle(ttl,'FontSize',12);
leg = legend("ref (bikesim)","est (Pryde model)",'FontSize',12);
leg.Orientation = "horizontal";
leg.Layout.Tile = 'south';

%% Save figure
dir = "C:\Users\marti\PhD\Thesis\MotorcycleDynamics\LinearModeling\Figures\";
saveas(fig,dir + "throttle_braking_results",'epsc');

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
    "$v_{x}$";
    "$\\omega_{r}$";
    "$\\omega_{f}$"
    ];

units = [
    "\\meter\\per\\second";
    "\\degree\\per\\second";
    "\\degree\\per\\second"
    ];

dir_name = "C:\Users\marti\PhD\Thesis\MotorcycleDynamics\LinearModeling\";

fid = fopen(dir_name + "throttle_braking_rmse_results.tex","w");
arrayfun(@(s)fprintf(fid,s),txt);
ftxt = @(x)"& " + num2str(x,'%.4f');

for k = 1:3
    txt = states(k) + " & $\\unit[per-mode=symbol]{" + units(k) + "}$ ";
    txt = txt + strjoin(arrayfun(ftxt,rmse_results(k,:)));
    txt = txt + "\\\\\n\\hline\n";
    fprintf(fid,txt);
end

fprintf(fid,"\\end{tabularx}");
fclose(fid);

percentage_rmse_results(isnan(percentage_rmse_results)) = 0;
percentage_rmse_results = mean(percentage_rmse_results,2)';

fid = fopen(dir_name + "throttle_braking_prmse_results.tex","w");
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