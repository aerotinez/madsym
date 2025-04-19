close("all"); clear; clc;
setmadsympath();

%% Figure setup
fig = figure('Position',[10,100,640,640]);

axe = axes(fig);
set(axe,'Color','none');

hold(axe,'on');
axis(axe,'equal');
axis(axe,'tight');
zlim(axe,[0,1.6]);
hold(axe,'off');

box(axe,'on');
light(axe);
view(axe,180,0);
xticks(axe,[]);
yticks(axe,[]);
zticks(axe,[]);
axe.XColor = 'none';
axe.YColor = 'none';
axe.ZColor = 'none';

% title(axe,'Chassis forces and moments','FontSize',24);

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
q = [-50,-30,-5,20];
% q = [0,0,0,0];
pitch_r = 12;
pitch_f = 12;
m = matlabColors;

% Rear tire
Ryaw = rotz(q(1));
Rcamber = Ryaw*rotx(q(2));
Rpitch_r = Rcamber*roty(pitch_r);
pr = rr.*Rcamber(:,3);
Tr = rigidtform3d(Rcamber,pr);

% Rear chassis
Rpitch = Rcamber*roty(q(3));
pb = tr*Ryaw(:,3) + (h - tr)*Rpitch(:,3) + b*Rpitch(:,1);
Tb = rigidtform3d(Rpitch,pb);

% Front chassis
Rcaster = Rpitch*roty(caster).';
p_caster = tr*Ryaw(:,3) - tr*Rpitch(:,3) + b*Rpitch(:,1) + a*Rcaster(:,1);
Rsteer = Rcaster*rotz(q(4));
ph = p_caster + e.*Rsteer(:,1) + f.*Rsteer(:,3);
Th = rigidtform3d(Rsteer,ph);

% Front tire
pf = p_caster + lx.*Rsteer(:,1) - lz.*Rsteer(:,3);
Tf = rigidtform3d(Rsteer*roty(caster),pf);

%% Plot frames
hr = plotFrame(axe,Tr,'Color',m.blue);
hb = plotFrame(axe,Tb,'Color',m.orange);
hh = plotFrame(axe,Th,'Color',m.yellow);
hf = plotFrame(axe,Tf,'Color',m.purple);

%% Legend
names = [
    "$\mathbf{T}_{r}$";
    "$\mathbf{T}_{b}$";
    "$\mathbf{T}_{h}$";
    "$\mathbf{T}_{f}$";
    ];

nmc = repelem("",3*numel(names));
nmc(1:3:3*numel(names)) = names;
nmc = cellstr(nmc);

leg = legend(axe,nmc{:}, ...
    'FontSize',12, ...
    'NumColumns',2, ...
    'Location','NorthEast', ...
    'Interpreter','latex', ...
    'IconColumnWidth',32, ...
    'AutoUpdate','off');

%% Plot and format motorcycle
mf = bigSportsFigure();
setPose(mf,q);

n0 = 0.75.*ones(1,3);
setColor(mf,n0,n0,n0,n0);
setAlpha(mf,0.25,0.25,0.25,0.25);

%% Plot force/moment axes

hold(axe,'on');

% steering axis
p0 = [
    0,0,0;
    0,0,1.5
    ];

T = rigidtform3d(Rcaster,p_caster);
p = transformPointsForward(T,p0);

plot3(axe,p(:,1),p(:,2),p(:,3),'k--','LineWidth',1.5);

% pitch axis
p0 = [
    0,0,0;
    0,0.75,0
    ];

T = rigidtform3d(Rpitch,pr);
p = transformPointsForward(T,p0);

plot3(axe,p(:,1),p(:,2),p(:,3),'k--','LineWidth',1.5);

% front pitch axis
p0 = [
    0,0,0;
    0,0.75,0
    ];

T = rigidtform3d(Rpitch,pb);
p = transformPointsForward(T,p0);

plot3(axe,p(:,1),p(:,2),p(:,3),'k--','LineWidth',1.5);

% front pitch axis
p0 = [
    0,0,0;
    0,0.75,0
    ];

T = rigidtform3d(Rsteer,pf);
p = transformPointsForward(T,p0);

plot3(axe,p(:,1),p(:,2),p(:,3),'k--','LineWidth',1.5);

% aero reference point
p0 = [
    0,0,0;
    0,0,h
    ];

T = rigidtform3d(Rpitch,pb - Rpitch*[0;0;h]);
p = transformPointsForward(T,p0);

plot3(axe,p(:,1),p(:,2),p(:,3),'k--','LineWidth',1.5);

p0 = [
    0,0,0;
    (a - an)/cosd(caster),0,0;
    ];

T = rigidtform3d(Rpitch,pb - Rpitch*[0;0;h]);
p = transformPointsForward(T,p0);

plot3(axe,p(:,1),p(:,2),p(:,3),'k--','LineWidth',1.5);

hold(axe,'off');
axis(axe,'tight');

%% Save figure
dir = 'C:\Users\marti\PhD\ThesisV2\MotorcycleDynamics\ForcesAndMoments\Figures\';
saveas(fig,string(dir) + "chassis_forces_unlabeled.eps",'epsc');