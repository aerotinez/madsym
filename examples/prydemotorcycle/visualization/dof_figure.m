close("all"); clear; clc;
setmadsympath();

%% Figure setup
w = 1920;
fig = figure('Position',[100,50,w,floor((148/210)*w)]);

axe = axes(fig);
set(axe,'Color','none');

hold(axe,'on');
axis(axe,'equal');
axis(axe,'tight');
zlim(axe,[0,1.6]);
hold(axe,'off');

box(axe,'on');
light(axe);
view(axe,143,13);
xticks(axe,[]);
yticks(axe,[]);
zticks(axe,[]);
axe.XColor = 'none';
axe.YColor = 'none';
axe.ZColor = 'none';

% title(axe,'Motorcycle degrees of freedom','FontSize',24);

%% Geometric parameters
caster = 24;
a = 749E-03;
an = 84.8E-03;
b = 572.9E-03;
d = 26.5E-03;
e = 28.7E-03;
f = 234.4E-03;
h = 575E-03;
rf = 290E-03;
tf = 50E-03;
rr = 290E-03;
tr = 70E-03;

s = sind(caster);
c = cosd(caster);

lx = rf*s - an;
lz = -(rf*c^2 + (an - a)*s)/c;

%% Place and orient frames
q = [-10,-10,1,30];
% q = [0,0,0,0];
pitch_r = 12;
pitch_f = 12;
m = matlabColors;

% World frame
I = eye(3);
p0 = [-0.7,0.5,0];
Tw = rigidtform3d(I,p0);

% Yaw frame
Ryaw = rotz(q(1));
pCr = [0,0,0];
Tyaw = rigidtform3d(Ryaw,pCr);

% Camber frame
Rcamber = Ryaw*rotx(q(2));
ptr = tr.*Ryaw(:,3);
Tcamber = rigidtform3d(Rcamber,ptr);

% Rear tire frame
Rpitch_r = Rcamber*roty(pitch_r);
pr = rr.*Rcamber(:,3);
Tpitch_r = rigidtform3d(Rpitch_r,pr);

% Pitch frame
Rpitch = Rcamber*roty(q(3));
Tpitch = rigidtform3d(Rpitch,pr);

% Rear chassis frame
pb = tr*Ryaw(:,3) + (h - tr)*Rpitch(:,3) + b*Rpitch(:,1);
Tb = rigidtform3d(Rpitch,pb);

% Caster frame
Rcaster = Rpitch*roty(caster).';
p_caster = tr*Ryaw(:,3) - tr*Rpitch(:,3) + b*Rpitch(:,1) + a*Rcaster(:,1);
Tcaster = rigidtform3d(Rcaster,p_caster);

% Front chassis frame
Rsteer = Rcaster*rotz(q(4));
ph = p_caster + e.*Rsteer(:,1) + f.*Rsteer(:,3);
Th = rigidtform3d(Rsteer,ph);

% Front tire frame
Rpitch_f = Rsteer*roty(caster)*roty(pitch_f);
pf = p_caster + lx.*Rsteer(:,1) - lz.*Rsteer(:,3);
Tf = rigidtform3d(Rpitch_f,pf);

% Front camber frame
Rf = Rsteer*roty(caster);
[yaw_f,camber_f,theta_f] = dcm2angle(Rf.','ZXY');
Rcamber_f = Rf*roty(rad2deg(theta_f)).';
ptf = pf - (rf - tf).*Rcamber_f(:,3);
Tcamber_f = rigidtform3d(Rcamber_f,ptf);

% Front yaw frame
Ryaw_f = Rcamber_f*rotx(rad2deg(camber_f)).';
pCf = ptf - tf.*Ryaw_f(:,3);
Tyaw_f = rigidtform3d(Ryaw_f,pCf);

%% Plot frames
hw = plotFrame(axe,Tw, ...
    'Color',[0,0,0]);

hyaw = plotFrame(axe,Tyaw, ...
    'Color',m.blue);

hcamber = plotFrame(axe,Tcamber, ...
    'Color',m.orange);

hpitch_r = plotFrame(axe,Tpitch_r, ...
    'Color',m.yellow);

hpitch = plotFrame(axe,Tpitch, ...
    'Color',m.purple);

hb = plotFrame(axe,Tb, ...
    'Color',m.green);

hcaster = plotFrame(axe,Tcaster, ...
    'Color',m.cyan);

hh = plotFrame(axe,Th, ...
    'Color',m.red);

hyaw_f = plotFrame(axe,Tyaw_f, ...
    'Color',m.blue, ...
    'LineStyle',':');

hcamber_f = plotFrame(axe,Tcamber_f, ...
    'Color',m.orange, ...
    'LineStyle',':');

hf = plotFrame(axe,Tf, ...
    'Color',m.yellow, ...
    'LineStyle',':');

%% Legend
names = [
    "$\mathbf{T}_{w}$";
    "$\mathbf{T}_{\psi}$";
    "$\mathbf{T}_{\gamma}$";
    "$\mathbf{T}_{\theta_{r},r}$";
    "$\mathbf{T}_{\theta}$";
    "$\mathbf{T}_{b}$";
    "$\mathbf{T}_{\varepsilon}$";
    "$\mathbf{T}_{\delta,h}$";
    "$\mathbf{T}_{\psi_{f}}$";
    "$\mathbf{T}_{\gamma_{f}}$";
    "$\mathbf{T}_{\theta_{f},f}$";
    ];

nmc = repelem("",3*numel(names));
nmc(1:3:3*numel(names)) = names;
nmc = cellstr(nmc);

leg = legend(axe,nmc{:}, ...
    'FontSize',32, ...
    'Position',[0.1,0.3,0.05,0.3], ...
    'Interpreter','latex', ...
    'IconColumnWidth',24, ...
    'AutoUpdate','off');


%% Plot and format motorcycle
mf = bigSportsFigure();
setPose(mf,q);

n0 = 0.75.*ones(1,3);
setColor(mf,n0,n0,n0,n0);
setAlpha(mf,0.25,0.25,0.25,0.25);

%% Plot dimension lines
plotDoFLine(axe, ...
    rigidtform3d(I,pr), ...
    rigidtform3d(I,[0,p0(2),pr(3)]), ...
    'y',[0.4,0]);

plotDoFLine(axe, ...
    rigidtform3d(I,pr), ...
    rigidtform3d(I,[p0(1),0,pr(3)]), ...
    'x',-[1,0]);

plotDoFLine(axe, ...
    rigidtform3d(I,pCr), ...
    rigidtform3d(Ryaw,pCr), ...
    'x',[1,1]);

plotDoFLine(axe, ...
    rigidtform3d(Ryaw,ptr), ...
    rigidtform3d(Rcamber,ptr), ...
    'y', ...
    [1,1]);

plotDoFLine(axe, ...
    rigidtform3d(Rcamber,pr), ...
    rigidtform3d(Rpitch_r,pr), ...
    'z', ...
    [1,1]);

plotDoFLine(axe, ...
    rigidtform3d(Rcamber,pr), ...
    rigidtform3d(Rpitch,pr), ...
    'z', ...
    [1,1]);

plotDoFLine(axe, ...
    rigidtform3d(Rcaster,ph), ...
    rigidtform3d(Rsteer,ph), ...
    'x',0.5.*[1,1]);

plotDoFLine(axe, ...
    rigidtform3d(I,pCf), ...
    rigidtform3d(Ryaw_f,pCf), ...
    'x',0.5.*[1,1]);

plotDoFLine(axe, ...
    rigidtform3d(Ryaw_f,ptf), ...
    rigidtform3d(Rcamber_f,ptf), ...
    'y',[1,1]);

plotDoFLine(axe, ...
    rigidtform3d(Rcamber_f,pf), ...
    rigidtform3d(Rpitch_f,pf), ...
    'z',[1,1]);

plotDoFLine(axe, ...
    rigidtform3d(I,[p0(1:2),pf(3)]), ...
    rigidtform3d(I,pf), ...
    'y', ...
    [-0.7,1.5]);

plotDoFLine(axe, ...
    rigidtform3d(I,[p0(1:2),pf(3)]), ...
    rigidtform3d(I,pf), ...
    'y', ...
    [0.5,0]);

plotDoFLine(axe, ...
    rigidtform3d(I,[p0(1:2),pf(3)]), ...
    rigidtform3d(I,pf), ...
    'x', ...
    [2.8,0.7]);

plotDoFLine(axe, ...
    rigidtform3d(I,[p0(1:2),pf(3)]), ...
    rigidtform3d(I,pf), ...
    'x', ...
    [-0.3,0]);

axis(axe,'tight');
camproj(axe,'perspective');

%% Save figure
dir = 'C:\Users\marti\PhD\Thesis\MotorcycleDynamics\RigidBodies\Figures\';
% saveas(fig,string(dir) + "motorcycle_dof_unlabled.eps",'epsc');