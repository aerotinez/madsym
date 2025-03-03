close("all"); clear; clc;
setmadsympath();

%% Figure setup
fig = figure('Position',[100,100,720,720]);

axe = axes(fig);
set(axe,'Color','none');

hold(axe,'on');
axis(axe,'equal');
axis(axe,'tight');
zlim(axe,[0,1.6]);
hold(axe,'off');

box(axe,'on');
light(axe);
view(axe,180,10);
xticks(axe,[]);
yticks(axe,[]);
zticks(axe,[]);
axe.XColor = 'none';
axe.YColor = 'none';
axe.ZColor = 'none';

title(axe,'Motorcycle degrees of freedom','FontSize',24);

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
q = [-45,-15,1,30];
% q = [0,0,0,0];
pitch_r = 12;
pitch_f = 12;
m = matlabColors;

% World frame
I = eye(3);
p0 = [-0.3,-0.3,0];
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
hw = plotFrame(axe,Tw,[0,0,0]);
hyaw = plotFrame(axe,Tyaw,m.blue);
hcamber = plotFrame(axe,Tcamber,m.orange);
hpitch_r = plotFrame(axe,Tpitch_r,m.yellow);
hpitch = plotFrame(axe,Tpitch,m.purple);
hb = plotFrame(axe,Tb,m.green);
hcaster = plotFrame(axe,Tcaster,m.cyan);
hh = plotFrame(axe,Th,m.red);
hyaw_f = plotFrame(axe,Tyaw_f,m.blue,':');
hcamber_f = plotFrame(axe,Tcamber_f,m.orange,':');
hf = plotFrame(axe,Tf,m.yellow,':');

%% Legend
names = [
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
    'FontSize',12, ...
    'NumColumns',5, ...
    'Location','South', ...
    'Interpreter','latex', ...
    'IconColumnWidth',32, ...
    'AutoUpdate','off');


%% Plot and format motorcycle
mf = bigSportsFigure();
setPose(mf,q);

n0 = 0.75.*ones(1,3);
setColor(mf,n0,n0,n0,n0);
setAlpha(mf,0.25,0.25,0.25,0.25);

%% Plot dimension lines
plotDoFLine(axe, ...
    rigidtform3d(I,p0), ...
    rigidtform3d(I,[0,-0.3,0]), ...
    'y',[1.8,1]);

plotDoFLine(axe, ...
    rigidtform3d(I,p0), ...
    rigidtform3d(I,[-0.3,0,0]), ...
    'x');

plotDoFLine(axe, ...
    rigidtform3d(I,pCr), ...
    rigidtform3d(Ryaw,pCr), ...
    'x');

plotDoFLine(axe, ...
    rigidtform3d(Ryaw,ptr), ...
    rigidtform3d(Rcamber,ptr), ...
    'y', ...
    [0.7,0.7]);

plotDoFLine(axe, ...
    rigidtform3d(Rcamber,pr), ...
    rigidtform3d(Rpitch_r,pr), ...
    'z', ...
    [1.5,1.5]);

plotDoFLine(axe, ...
    rigidtform3d(Rcamber,pr), ...
    rigidtform3d(Rpitch,pr), ...
    'z', ...
    [1.5,1.5]);

plotDoFLine(axe, ...
    rigidtform3d(Rcaster,ph), ...
    rigidtform3d(Rsteer,ph), ...
    'x');

plotDoFLine(axe, ...
    rigidtform3d(I,pCf), ...
    rigidtform3d(Ryaw_f,pCf), ...
    'x');

plotDoFLine(axe, ...
    rigidtform3d(Ryaw_f,ptf), ...
    rigidtform3d(Rcamber_f,ptf), ...
    'y');

plotDoFLine(axe, ...
    rigidtform3d(Rcamber_f,pf), ...
    rigidtform3d(Rpitch_f,pf), ...
    'z');

plotDoFLine(axe, ...
    rigidtform3d(I,p0), ...
    rigidtform3d(I,pCf), ...
    'y', ...
    [1,2]);

plotDoFLine(axe, ...
    rigidtform3d(I,p0), ...
    rigidtform3d(I,pCf), ...
    'x', ...
    [2,1]);

axis(axe,'tight');
camproj(axe,'perspective');