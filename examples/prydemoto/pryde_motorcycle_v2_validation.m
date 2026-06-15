close("all"); clear; clc;
setmadsympath();

%% Parameters
bs = bigSportsParameters;
s2m = @(x) cell2mat(struct2cell(x));

prydeParams = @(v) s2m(bikeSimToPrydeV2Parameters(bs, v/3.6));
sharpStruct30 = bikeSimToSharpParameters(bs, 30/3.6);
sharpParamsStruct = @(v) bikeSimToSharpParameters(bs, v/3.6);
sharpParams = @(v) s2m(sharpParamsStruct(v));

%% General settings
vx = [30, 50, 80, 110, 130];
n2s = @num2str;

scen = "OpenLoop";
results_path = "G:\My Drive\BikeSimResults\BigSports\" + scen;

prydePlant = @prydeMotoLateralStateSpace;
sharpPlant = @sharpMotorcycleStateSpace;

ts = 1/60;

% Storage
x_ref   = cell(1, numel(vx));
x_pryde = cell(1, numel(vx));
x_sharp = cell(1, numel(vx));

%% Sharp geometry term
l = (sharpStruct30.a - sharpStruct30.an)/cos(sharpStruct30.caster);

%% Simulate both models
for k = 1:numel(vx)
    speed_path = "\Vx" + n2s(vx(k)) + "Kph\";
    file_name = "bikesim_results_" + n2s(vx(k)) + "kph.csv";
    results = readtable(results_path + speed_path + file_name);

    time = results.Time;
    Mz = results.M_Str_In;

    %% =========================
    % Pryde reference and model
    % States: [camber steer wz vy wx ws ar af]
    %% =========================
    yaw   = deg2rad(results.Yaw);
    pitch = deg2rad(results.Pitch);
    roll  = deg2rad(results.Roll_E);
    R = angle2dcm(yaw, pitch, roll);
    [~, camber_pr, ~] = dcm2angle(R, 'ZXY');
    camber_pr = rad2deg(camber_pr);

    steer_pr = results.Steer;
    wz_pr    = results.AVz;
    vy_pr    = results.VyW0_2 ./ 3.6;
    wx_pr    = results.AVx;
    ws_pr    = -results.M_StrSys ./ 0.2212;
    ar_ref   = results.Alpha_2;
    af_ref   = results.Alpha_1;

    x_meas_pryde = [camber_pr, steer_pr, wz_pr, vy_pr, wx_pr, ws_pr, ar_ref, af_ref];

    idxP = [1, 1, 1, 0, 1, 1, 1, 1];
    sP   = (180/pi).*idxP + not(idxP);
    sfP  = (pi/180).*idxP + not(idxP);
    ICP  = sfP .* x_meas_pryde(1, :);

    sysP = prydePlant(prydeParams(vx(k)));
    x_sim_pryde = sP .* lsim(sysP, Mz, time, ICP);

    %% =========================
    % Sharp reference and model
    % Original Sharp states: [camber steer wz vy wx ws Yr Yf]
    % Convert to common convention and convert Sharp vy reference point
    % to Pryde reference point.
    %% =========================
    pS_struct = sharpParamsStruct(vx(k));
    pS = sharpParams(vx(k));

    % Angular velocity and translational velocity from BikeSim
    w_rad = deg2rad(table2array(results(:, ["AVx", "AVy", "AVz"])));
    v_lin = table2array(results(:, ["Vxo", "Vyo", "Vzo"])) / 3.6;

    % Velocity at Sharp model reference point
    r_sharp_from_velref = [-l, 0, 0];
    va = cross(w_rad, r_sharp_from_velref .* ones(size(w_rad))) + v_lin;

    % Raw Sharp/native convention measurements
    camber_sh_model = results.Roll;
    steer_sh_model  = results.Steer;
    wz_sh_model     = results.AVz;
    wx_sh_model     = results.AVx;
    ws_sh_model     = -results.M_StrSys ./ 0.2212;
    Yr_ref          = results.Fy_2;
    Yf_ref          = results.Fy_1;

    % Convert Sharp vy reference point -> Pryde reference point
    % r = [b, 0, -rr], and:
    % vy_pryde = vy_sharp - (wz*b + wx*rr)
    vy_sharp_ref = va(:,2);  % m/s, at Sharp reference point
    vy_pryde_ref = vy_sharp_ref ...
        - (deg2rad(wz_sh_model) .* pS_struct.b + deg2rad(wx_sh_model) .* pS_struct.rr);

    % Convert to common comparison convention
    camber_sh = -camber_sh_model;
    steer_sh  = steer_sh_model;
    wz_sh     = wz_sh_model;
    vy_sh     = vy_pryde_ref;
    wx_sh     = -wx_sh_model;
    ws_sh     = ws_sh_model;

    % Infer slip angles using native Sharp camber in the tire law
    ar_sh_ref = (Yr_ref - pS_struct.Cr2 .* camber_sh_model) ./ pS_struct.Cr1;
    af_sh_ref = (Yf_ref - pS_struct.Cf2 .* ...
        (sin(pS_struct.caster) .* steer_sh_model + camber_sh_model)) ./ pS_struct.Cf1;

    % Sharp states in common comparison convention:
    % [camber steer wz vy wx ws ar af]
    x_meas_sharp = [camber_sh, steer_sh, wz_sh, vy_sh, wx_sh, ws_sh, ar_sh_ref, af_sh_ref];

    % Initial conditions must remain in native Sharp convention and native reference point
    idxS = [1, 1, 1, 0, 1, 1, 0, 0];
    sS   = (180/pi).*idxS + not(idxS);
    sfS  = (pi/180).*idxS + not(idxS);
    ICS  = sfS .* [ ...
        camber_sh_model(1), ...
        steer_sh_model(1), ...
        wz_sh_model(1), ...
        vy_sharp_ref(1), ...
        wx_sh_model(1), ...
        ws_sh_model(1), ...
        Yr_ref(1), ...
        Yf_ref(1)];

    sysS = sharpPlant(pS);
    x_sim_sharp_raw = sS .* lsim(sysS, Mz, time, ICS);   % native Sharp convention and reference point

    % Native simulated outputs
    camber_sim_sh_model = x_sim_sharp_raw(:,1);   % deg
    steer_sim_sh_model  = x_sim_sharp_raw(:,2);   % deg
    wz_sim_sh_model     = x_sim_sharp_raw(:,3);   % deg/s
    vy_sim_sharp_ref    = x_sim_sharp_raw(:,4);   % m/s, Sharp ref point
    wx_sim_sh_model     = x_sim_sharp_raw(:,5);   % deg/s
    ws_sim_sh_model     = x_sim_sharp_raw(:,6);   % deg/s
    Yr_sim              = x_sim_sharp_raw(:,7);   % N
    Yf_sim              = x_sim_sharp_raw(:,8);   % N

    % Convert simulated Sharp vy reference point -> Pryde reference point
    vy_sim_pryde_ref = vy_sim_sharp_ref ...
        - (deg2rad(wz_sim_sh_model) .* pS_struct.b + deg2rad(wx_sim_sh_model) .* pS_struct.rr);

    % Convert simulated outputs to common comparison convention
    camber_sim_sh = -camber_sim_sh_model;
    steer_sim_sh  = steer_sim_sh_model;
    wz_sim_sh     = wz_sim_sh_model;
    vy_sim_sh     = vy_sim_pryde_ref;
    wx_sim_sh     = -wx_sim_sh_model;
    ws_sim_sh     = ws_sim_sh_model;

    % Reconstruct slip angles using native Sharp camber in the tire law
    ar_sh_sim = (Yr_sim - pS_struct.Cr2 .* camber_sim_sh_model) ./ pS_struct.Cr1;
    af_sh_sim = (Yf_sim - pS_struct.Cf2 .* ...
        (sin(pS_struct.caster) .* steer_sim_sh_model + camber_sim_sh_model)) ./ pS_struct.Cf1;

    x_sim_sharp = [ ...
        camber_sim_sh, ...
        steer_sim_sh, ...
        wz_sim_sh, ...
        vy_sim_sh, ...
        wx_sim_sh, ...
        ws_sim_sh, ...
        ar_sh_sim, ...
        af_sh_sim];

    %% =========================
    % Store results
    %% =========================
    % Use Pryde reference as the common reference set
    x_ref{k}   = x_meas_pryde;
    x_pryde{k} = x_sim_pryde;
    x_sharp{k} = x_sim_sharp;
end

%% Plot results
fig = figure("Position", [1, 50, 840, 840]);

tl = tiledlayout(8, 5, ...
    "Parent", fig, ...
    "Padding", "compact", ...
    "TileSpacing", "tight");

titles = arrayfun(@(x) "Speed: " + x + " km/h", vx);

units = [
    "\gamma (\circ)";
    "\delta (\circ)";
    "\omega_z (\circ/s)";
    "v_y (m/s)";
    "\omega_x (\circ/s)";
    "\omega_\delta (\circ/s)";
    "\alpha_r (\circ)";
    "\alpha_f (\circ)"
    ];

uind = (0:5:35) + 1;
j = 1;

% RMSE and normalized RMSE storage
rmse_pryde = zeros(8, numel(vx));
rmse_sharp = zeros(8, numel(vx));
nrmse_pryde = zeros(8, numel(vx));
nrmse_sharp = zeros(8, numel(vx));

for k = 1:8*5
    [row, col] = ind2sub([5, 8], k);
    ax = nexttile(tl, k);
    hold(ax, "on");

    ns = size(x_ref{row}, 1);
    time = linspace(0, (ns - 1)*ts, ns);

    plot(ax, time, x_ref{row}(:, col),   "Color", [0,0,0], "LineWidth", 1.5);
    plot(ax, time, x_sharp{row}(:, col), '-.', "Color", matlabColors().blue, "LineWidth", 1.5);
    plot(ax, time, x_pryde{row}(:, col), '-.', "Color", matlabColors().orange, "LineWidth", 1.5);

    hold(ax, "off");
    xlim(ax, [0, time(end)]);
    box(ax, "on");
    axis(ax, "tight");

    if k < 6
        title(ax, titles(k), "FontSize", 12);
    end

    if ismember(k, uind)
        ylabel(ax, units(j), "Interpreter", "tex", "FontSize", 12);
        j = j + 1;
    end

    if k <= 35
        xticks(ax, []);
    else
        xlabel(ax, "time (s)", "FontSize", 12);
    end

    rmse_pryde(col, row) = rmse(x_pryde{row}(:, col), x_ref{row}(:, col));
    rmse_sharp(col, row) = rmse(x_sharp{row}(:, col), x_ref{row}(:, col));

    ref_span = max(x_ref{row}(:, col)) - min(x_ref{row}(:, col));

    if ref_span > 1e-8
        nrmse_pryde(col, row) = 100 * rmse_pryde(col, row) / ref_span;
        nrmse_sharp(col, row) = 100 * rmse_sharp(col, row) / ref_span;
    else
        nrmse_pryde(col, row) = NaN;
        nrmse_sharp(col, row) = NaN;
    end
end

mean_nrmse_pryde = mean(nrmse_pryde, 2, "omitnan");
mean_nrmse_sharp = mean(nrmse_sharp, 2, "omitnan");

dict = dictionary(["OpenLoop","DLC","Chicane"], ...
                  ["Open loop","Double Lane-Change","Chicane"]);

ttl = "Lateral motorcycle model results: " + dict(scen);
sgtitle(ttl, "FontSize", 12);

leg = legend("BikeSim", "Sharp", "Pryde", "FontSize", 12);
leg.Orientation = "horizontal";
title(leg,"Model","FontSize",12,"FontWeight","bold");
leg.Layout.Tile = "south";

%% Save figure
dict = dictionary(["OpenLoop","DLC","Chicane"],["open_loop","dlc","chicane"]);

dir = "C:\Users\marti\PhD\Articles\VSD26\Figures\";
saveas(fig,dir + dict(scen) + "_results.eps",'epsc');

%% RMSE table
states = [
    "$\gamma$";
    "$\delta$";
    "$\omega_{z}$";
    "$v_{y}$";
    "$\omega_{x}$";
    "$\omega_{\delta}$";
    "$\alpha_{r}$";
    "$\alpha_{f}$"
    ];

latex_units = [
    "\degree";
    "\degree";
    "\degree\per\second";
    "\meter\per\second";
    "\degree\per\second";
    "\degree\per\second";
    "\degree";
    "\degree"
    ];

table_dir = "C:\Users\marti\PhD\Articles\VSD26\";

fid = fopen(table_dir + dict(scen) + "_rmse_results.tex","w");

fprintf(fid,"\\begin{tabularx}{\\textwidth}{|C|C||*{6}{C|}}\n");
fprintf(fid,"\\hline\n");
fprintf(fid,"\\multirow{2}{*}{State} &\n");
fprintf(fid,"\\multirow{2}{*}{Units} &\n");
fprintf(fid,"\\multicolumn{5}{c|}{Speed ($\\unit[per-mode=symbol]{\\kilo\\meter\\per\\hour}$)} &\n");
fprintf(fid,"\\multirow{2}{*}{$\\overline{\\text{RMSE\\percent}}$} \\\\\n");
fprintf(fid,"\\cline{3-7}\n");
fprintf(fid,"& & 30 & 50 & 80 & 110 & 130 & \\\\\n");
fprintf(fid,"\\hline\\hline\n");

for k = 1:8
    fprintf(fid,"%s & $\\unit[per-mode=symbol]{%s}$ ", states(k), latex_units(k));
    fprintf(fid,"& %.4f & %.4f & %.4f & %.4f & %.4f & %.2f ", ...
        rmse_pryde(k,:), mean_nrmse_pryde(k));
    fprintf(fid,"\\\\\n\\hline\n");
end

fprintf(fid,"\\end{tabularx}\n");
fclose(fid);