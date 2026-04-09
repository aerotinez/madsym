close("all"); clear; clc;
setmadsympath();

%% Parameters
bs = bigSportsParameters;
s2m = @(x)cell2mat(struct2cell(x));
params = @(v)s2m(bikeSimToPrydeV2Parameters(bs,v/3.6));

%% Model sweep
nv = 3000;
vmin = 30;
vmax = 130;
vx = linspace(vmin,vmax,nv);

plant = @prydeMotoLateralStateSpace;
sys = arrayfun(@(v)plant(params(v)), vx.', "uniform", 0);

%% --- TRACK MODES (eig + eigenvector MAC) ---
nV = numel(sys);

[A0,~] = ssdata(sys{1});
[V,D] = eig(A0,"vector");

% sort initial poles
[~,idx] = sortrows([abs(imag(D)) real(D)]);
D = D(idx);
V = V(:,idx);

nP = numel(D);
lam = zeros(nP,nV);
vec = zeros(size(V,1),nP,nV);

lam(:,1) = D;
vec(:,:,1) = V;

for k = 2:nV
    [Aprev,~] = ssdata(sys{k-1});
    [Acurr,~] = ssdata(sys{k});

    [Vprev,Dprev] = eig(Aprev,"vector");
    [Vcurr,Dcurr] = eig(Acurr,"vector");

    % previous tracked
    Dp = lam(:,k-1);
    Vp = vec(:,:,k-1);

    % normalize
    for i = 1:nP
        Vp(:,i) = Vp(:,i)/norm(Vp(:,i));
        Vcurr(:,i) = Vcurr(:,i)/norm(Vcurr(:,i));
    end

    % cost matrix
    C = zeros(nP,nP);
    scale = max(1, median(abs(Dcurr)));

    for i = 1:nP
        for j = 1:nP
            eigDist = abs(Dp(i) - Dcurr(j)) / scale;
            mac = abs(Vp(:,i)'*Vcurr(:,j))^2;
            vecDist = 1 - mac;
            C(i,j) = eigDist + vecDist;
        end
    end

    % assignment (greedy, good enough)
    order = zeros(nP,1);
    used = false(nP,1);

    for i = 1:nP
        [~,j] = min(C(i,:) + 1e6*used.');
        order(i) = j;
        used(j) = true;
    end

    lam(:,k) = Dcurr(order);
    vec(:,:,k) = Vcurr(:,order);
end

%% --- CLASSIFY MODES ---
fd = abs(imag(lam))/(2*pi);

weave = zeros(1,nV);
wobble = zeros(1,nV);
capsize = zeros(1,nV);

for k = 1:nV
    l = lam(:,k);

    realMask = abs(imag(l)) < 1e-6;
    complexMask = imag(l) > 0;

    % capsize: real pole with largest real part
    r = find(realMask);
    if ~isempty(r)
        [~,i] = max(real(l(r)));
        capsize(k) = r(i);
    end

    % complex modes
    c = find(complexMask);
    if numel(c) >= 2
        [~,ord] = sort(abs(imag(l(c))));
        weave(k) = c(ord(1));
        wobble(k) = c(ord(end));
    end
end

%% --- FREQUENCIES ---
f_weave  = arrayfun(@(k) fd(weave(k),k), 1:nV);
f_wobble = arrayfun(@(k) fd(wobble(k),k),1:nV);

%% --- PLOT: pole trajectories ---
fig = figure("Position",[100,100,640,240]); 
axe = axes(fig);
cm = parula(nV);

m = matlabColors;
c = [
    m.blue
    m.blue
    m.orange
    m.yellow
    m.purple
    ];
lam = lam([1,2,3,5,7],:);

xlabel(axe,"Re (1/s)","FontSize",12);
ylabel(axe,"Im (1/s)","FontSize",12);
title(axe,"Lateral model: poles vs forward speed","FontSize",12);
box(axe,"on");
hold(axe,"on");

sf = 2;

for i = 1:5
    x = real(lam(i,:));
    y = imag(lam(i,:));

    scatter(axe,x,y,sf*75,c(i,:),"filled", ...
        "MarkerFaceColor",c(i,:));

    scatter(axe,x,y,sf*10,cm,"filled");
end

sgrid(axe);

cb = colorbar(axe);
f = @(x)string((vmax - vmin)*double(string(x)) + vmin);
cb.TickLabels = cellfun(f,cb.TickLabels,'uniform',0);
cb.Label.String = "Speed (km/h)";
cb.Label.FontSize = 12;

nms = [
    "";
    "";
    "";
    "";
    "Capsize";
    "";
    "Weave";
    "";
    "Relaxation";
    "";
    "Wobble";
    ""
    ];

leg = legend(nms, ...
    "Location","northwest", ...
    "FontSize",12);

title(leg,"Mode","FontSize",12);

hold(axe,"off");
% saveas(fig,"C:/Users/marti/PhD/Articles/VSD26/Figures/lateral_nyquist",'epsc');
