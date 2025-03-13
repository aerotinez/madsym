close("all"); clear; clc;
setmadsympath();

%% Figure setup
fig = figure('Position',[10,100,480,640]);

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

m = matlabColors;

%%
q = [0,0,0,0];
% q = [0,0,0,0];

rf = 290E-03;
tf = 50E-03;
rr = 290E-03;
tr = 50E-03;

%% 
mf = bigSportsFigure();
setPose(mf,q);

n0 = 0.75.*ones(1,3);
setColor(mf,n0,n0,n0,n0);
setAlpha(mf,0.5,0,0,0);
delete(axe.Children(1:end - 2));
axis(axe,'tight');

%%

I = eye(3);
p0 = [0,0,0];

Ryaw = rotz(q(1));
hCr = plotFrame(axe,rigidtform3d(Ryaw,[0,0,0]), ...
    'Color',m.blue, ...
    "Scale",0.25);

Rcamber = Ryaw*rotx(q(2));
ptr = tr.*Ryaw(:,3);
htr = plotFrame(axe,rigidtform3d(Rcamber,ptr), ...
    'Color',m.orange, ...
    "Scale",0.25);

Rpitch = Rcamber*roty(22.5);
prr = ptr + (rr - tr).*Rcamber(:,3);
h = plotFrame(axe,rigidtform3d(Rpitch,prr), ...
    'Color',m.yellow, ...
    "Scale",0.25);

hold(axe,'on');
plot3(axe,rr.*[-1,1],[0,0],[0,0],'k','LineWidth',10);
hold(axe,'off');

%% Save figure
dir = 'C:\Users\marti\PhD\Thesis\MotorcycleDynamics\ForcesAndMoments\Figures\';
saveas(fig,string(dir) + "rolling_resistance_unlabeled.eps",'epsc');