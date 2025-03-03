close("all"); clear; clc;
setmadsympath();

%% Figure setup
fig = figure('Position',[10,270,640,640]);

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

title(axe,'Front contact point constraint','FontSize',24);

%% Solve for pitch
yaw = 0;
camber = 35;
steer = 15;

Ryaw = rotz(yaw);
Rcamber = Ryaw*rotx(camber);
