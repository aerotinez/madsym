close("all"); clear; clc;
setmadsympath();

%% Experiment parameters
ns = 101;
vx = linspace(30,130,ns)/3.6;
wz = deg2rad(linspace(-20,20,ns));
g = 9.81;

%% Parameters
bs = bigSportsParameters;

fpacejka = @(p)[
    p.fz0;
    p.pkx1;
    p.pkx2;
    p.pkx3;
    p.pky1;
    p.pky2;
    p.pky3;
    p.pky6;
    p.pky7;
    p.qdz1;
    p.qdz2;
    p.qdz8;
    p.qdz9
    ];

pf = fpacejka(bs.FrontTire.Pacejka);
pr = fpacejka(bs.RearTire.Pacejka);

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

    p = P{idx_vx};

    idx_branches = {
        idx0:ns
        idx0:-1:1
        };

    for bb = 1:numel(idx_branches)
        xg = x0;

        for kk = idx_branches{bb}
            f = @(x)steadyTurningTrimFun(x,[wz(kk);vx(idx_vx)],p,pf,pr);

            X(kk,idx_vx,:) = fsolve(f,xg,opts);
            xg = squeeze(X(kk,idx_vx,:));
        end
    end
end

X(:,:,1:4) = rad2deg(X(:,:,1:4));
X(:,:,5:6) = 100*X(:,:,5:6);

%% Plot

[Vx,Wz] = meshgrid(vx,wz);
Wz = rad2deg(Wz);

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
    "angle (\circ)";
    "angle (\circ)";
    "angle (\circ)";
    "angle (\circ)";
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

    surf(axe,Wz,3.6*Vx,X(:,:,k),"EdgeColor","none");
    [xv,yv,zv] = surfnorm(Wz,3.6*Vx,X(:,:,k));

    n = [
        mean(xv(:),"omitnan")
        mean(yv(:),"omitnan")
        mean(zv(:),"omitnan")
        ];

    [az,el] = viewFromNormal(n);
    view(axe,az + 45,el);

    title(axe,titles(k),"FontSize",12);
    xlabel(axe,"Yaw rate (\circ/s)","FontSize",12);
    ylabel(axe,"Speed (kph)","FontSize",12);
    zlabel(axe,units(k),"FontSize",12);
    axis(axe,"square");
    box(axe,"on");
    camproj(axe,"perspective");
end

sgtitle(tl,"Trim parameter sweep","FontSize",12);

%% Helpers
function [F,J] = steadyTurningTrimFun(x,u,p,pf,pr)
    F = steadyTurningTrimEqns(x,u,p,pf,pr);
    J = steadyTurningTrimJac(x,u,p,pf,pr);
end

function [az,el] = viewFromNormal(n)
    n = n./norm(n);
    az = atan2d(n(2),n(1)) + 180;
    el = asind(n(3));
    el = max(min(el,45),15);
end