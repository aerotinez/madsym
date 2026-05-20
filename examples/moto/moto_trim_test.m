close("all"); clear; clc;
setmadsympath();

%% Experiment parameters
ns = 100;
vx = linspace(30,130,ns)/3.6;
r = -linspace(150,1E03,ns);
kappa = 1./r;
g = 9.81;

%% Parameters
bs = bigSportsParameters;
P = cell(ns,1);

for idx_vx = 1:ns
    params = bikeSimToMotoParameters(bs,vx(idx_vx));
    P{idx_vx} = cell2mat(struct2cell(params));
end

%% Solver
opts = optimoptions("fsolve", ...
    "SpecifyObjectiveGradient",true, ...
    "Display","none");

%% Sweep
X = zeros(ns,ns,13);

idx0 = ceil(ns/2);

for idx_vx = 1:ns
    params = bikeSimToMotoParameters(bs,vx(idx_vx));

    x0 = [
        0;
        zeros(6,1);
        vx(idx_vx)/params.rf;
        vx(idx_vx)/params.rr;
        params.fzf;
        params.fzr;
        0;
        0
        ];

    idx_seq = ns:-1:1;

    xg = x0;

    for kk = idx_seq
        wz = vx(idx_vx)*kappa(kk);
        p = P{idx_vx};

        f = @(x)steadyTurningTrimFun(x,[wz;vx(idx_vx)],p);

        X(kk,idx_vx,:) = fsolve(f,xg,opts);
        xg = squeeze(X(kk,idx_vx,:));
    end
end

X(:,:,1:4) = rad2deg(X(:,:,1:4));
X(:,:,5:6) = 100*X(:,:,5:6);

%% Plot

[Vx,R] = meshgrid(vx,r);
Wz = Vx./R;

fig = figure;

titles = [
    "Camber";
    "Steer";
    "Front slip angle";
    "Rear slip angle";
    "Front slip ratio";
    "Rear slip ratio";
    "Side speed";
    "Front wheel speed";
    "Rear wheel speed";
    "Front normal force";
    "Rear normal force";
    "Throttle torque";
    "Steering torque"
    ];

units = [
    "angle (deg)";
    "angle (deg)";
    "angle (deg)";
    "angle (deg)";
    "ratio (%)";
    "ratio (%)";
    "speed (m/s)";
    "speed (rad/s)";
    "speed (rad/s)";
    "force (N)";
    "force (N)";
    "torque (Nm)";
    "torque (Nm)"
    ];

tl = tiledlayout(3,5,"TileSpacing","tight","Padding","compact");

for k = 1:numel(titles)
    axe = nexttile(tl,k);
    surf(axe,R,3.6*Vx,X(:,:,k),"EdgeColor","none");
    title(axe,titles(k),"FontSize",12);
    xlabel(axe,"Turn radius (m)","FontSize",12);
    ylabel(axe,"Speed (kph)","FontSize",12);
    zlabel(axe,units(k),"FontSize",12);
    axis(axe,"square");
    box(axe,"on");
end

sgtitle(tl,"Trim parameter sweep","FontSize",12);

%% Helpers
function [F,J] = steadyTurningTrimFun(x,u,p)
    F = steadyTurningTrimEqns(x,u,p);
    J = steadyTurningTrimJac(x,u,p);
end