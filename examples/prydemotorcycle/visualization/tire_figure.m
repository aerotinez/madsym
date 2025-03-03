close("all"); clear; clc;
setmadsympath();

%% Figure setup
fig = figure('Position',[10,270,480,640]);

axe = axes(fig);
set(axe,'Color','none');

hold(axe,'on');
axis(axe,'equal');
axis(axe,'tight');
zlim(axe,[0,1.6]);
hold(axe,'off');

box(axe,'on');
light(axe);
view(axe,180,30);
xticks(axe,[]);
yticks(axe,[]);
zticks(axe,[]);
axe.XColor = 'none';
axe.YColor = 'none';
axe.ZColor = 'none';

m = matlabColors;

title(axe,'Motorcycle tire dynamics','FontSize',24);

%%
q = [-50,-10,12,0];
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
hw = plotFrame(axe,rigidtform3d(rotz(-60),p0),p0);

Ryaw = rotz(q(1));
hCr = plotFrame(axe,rigidtform3d(Ryaw,[0,0,0]),m.blue);

Rcamber = Ryaw*rotx(q(2));
ptr = tr.*Ryaw(:,3);
htr = plotFrame(axe,rigidtform3d(Rcamber,ptr),m.orange);

Rpitch = Rcamber*roty(q(3));
prr = ptr + (rr - tr).*Rcamber(:,3);
h = plotFrame(axe,rigidtform3d(Rpitch,prr),m.yellow);

%%

plotDoFLine(axe, ...
    rigidtform3d(rotz(-60),p0), ...
    rigidtform3d(Ryaw,p0), ...
    'y',0.5.*[1,1]);

plotDoFLine(axe, ...
    rigidtform3d(Ryaw,ptr), ...
    rigidtform3d(Rcamber,ptr), ...
    'z',[0.8,0.7]);

plotDoFLine(axe, ...
    rigidtform3d(Rcamber,prr), ...
    rigidtform3d(Rpitch,prr), ...
    'x',0.5.*[1,1]);

plotDoFLine(axe, ...
    rigidtform3d(Rcamber,prr), ...
    rigidtform3d(Rpitch,prr), ...
    'y',[0,0.5]);

plotDoFLine(axe, ...
    rigidtform3d(Ryaw,p0), ...
    rigidtform3d(Ryaw*rotz(10),p0), ...
    'x',[0.6,0.45]);

