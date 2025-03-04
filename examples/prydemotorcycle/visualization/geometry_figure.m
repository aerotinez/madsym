close("all"); clear; clc;
setmadsympath();

%% Figure setup
fig = figure('Position',[100,100,720,640]);

axe = axes(fig);
set(axe,'Color','none');

hold(axe,'on');
axis(axe,'equal');
axis(axe,'tight');
zlim(axe,[0,2]);
hold(axe,'off');

box(axe,'on');
light(axe);
view(180,0);
xticks(axe,[]);
yticks(axe,[]);
zticks(axe,[]);
axe.XColor = 'none';
axe.YColor = 'none';
axe.ZColor = 'none';

% title(axe,'Motorcycle geometry','FontSize',24);

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


%% Plot and format motorcycle
q = [0,0,0,0];
m = matlabColors;

mf = bigSportsFigure();
setPose(mf,q);
setColor(mf,m.blue,m.orange,m.yellow,m.purple);
setAlpha(mf,0.25,0.25,0.25,0.25);

%% Legend

names = {
    'Rear tire';
    'Rear chassis';
    '';
    '';
    'Front chassis';
    '';
    '';
    '';
    'Front tire';
    '';
    '';
    '';
    '';
    ''
    };

legend(axe,names{:}, ...
    'Fontsize',12, ...
    'Location','SouthOutside', ...
    'Orientation','Horizontal', ...
    'AutoUpdate','off');

%% Dimension lines

R = [
    cosd(caster),-sind(caster);
    sind(caster),cosd(caster)
    ];

hold(axe,'on');

% Ground
plot3(axe,[-0.5,1.7],[0,0],[0,0],'k','LineWidth',3);

% Cr
plot3(axe,[0,0],[0,0],[0,1.9],'k--','LineWidth',1.5);

% Cf
plot3(axe,[1.3,1.3],[0,0],[0,1.9],'k--','LineWidth',1.5);

% a
p = R*[
    0,0;
    0,1;
    ] + [b,0].';

plot3(axe,p(1,:),[0,0],p(2,:),'k--','LineWidth',1.5);

% an
p = R*[
    0,0;
    0,1.1;
    ] + [1.3,0].';

plot3(axe,p(1,:),[0,0],p(2,:),'k--','LineWidth',1.5);

% b
plot3(axe,[b,b],[0,0],[0,1.7],'k--','LineWidth',1.5);

% caster
p = R*[
    0,1.3;
    0,0
    ] + [b,0].';

plot3(axe,p(1,:),[0,0],p(2,:),'k--','LineWidth',1.5);

% e
p = R*[
    0,0;
    0,0.92
    ] + [1.3,rf].' + R*[e - lx + 0.05,0].';

plot3(axe,p(1,:),[0,0],p(2,:),'k--','LineWidth',1.5);

% f
p = R*[
    0,0.5;
    0,0
    ] + [1.3,rf].' + R*[0,f + lz].';

plot3(axe,p(1,:),[0,0],p(2,:),'k--','LineWidth',1.5);

% h
p = [
    0,-0.4;
    0,0
    ] + [b,h].';

plot3(axe,p(1,:),[0,0],p(2,:),'k--','LineWidth',1.5);

% lx
p = R*[
    0,0;
    0,1
    ] + [1.3,rf].';

plot3(axe,p(1,:),[0,0],p(2,:),'k--','LineWidth',1.5);

% lz
p = R*[
    0,0.5;
    0,0
    ] + [1.3,rf].';

plot3(axe,p(1,:),[0,0],p(2,:),'k--','LineWidth',1.5);

% steering axis
p = R*[
    0,0;
    0,1
    ] + [1.3,rf].' + R*[-lx,0].';

plot3(axe,p(1,:),[0,0],p(2,:),'k--','LineWidth',1.5);

% rf
p = [
    0,0.4;
    0,0
    ] + [1.3,rf].';

plot3(axe,p(1,:),[0,0],p(2,:),'k--','LineWidth',1.5);

% tf
p = [
    0,0.3;
    0,0
    ] + [1.3,tf].';

plot3(axe,p(1,:),[0,0],p(2,:),'k--','LineWidth',1.5);

% rr
p = [
    0,-0.4;
    0,0
    ] + [0,rr].';

plot3(axe,p(1,:),[0,0],p(2,:),'k--','LineWidth',1.5);

% tr
p = [
    0,-0.3;
    0,0
    ] + [0,tr].';

plot3(axe,p(1,:),[0,0],p(2,:),'k--','LineWidth',1.5);

hold(axe,'off');

%% Save figure
dir = 'C:\Users\marti\PhD\Thesis\MotorcycleDynamics\RigidBodies\Figures\';
saveas(fig,string(dir) + "motorcycle_geometry_unlabled.eps",'epsc');