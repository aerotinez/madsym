close("all"); clear; clc;
setmadsympath();

%% Figure setup
fig = figure('Position',[10,100,480,640]);

axe = axes(fig);
set(axe,'Color','none');

hold(axe,'on');
axis(axe,'equal');
axis(axe,'tight');
hold(axe,'off');

box(axe,'on');
light(axe);
view(axe,-90,0);
xticks(axe,[]);
yticks(axe,[]);
zticks(axe,[]);
axe.XColor = 'none';
axe.YColor = 'none';
axe.ZColor = 'none';

m = matlabColors;

% title(axe,'Motorcycle tire dynamics','FontSize',24);

%%
q = [0,-30,0,0];
% q = [0,0,0,0];

rf = 290E-03;
tf = 50E-03;
rr = 290E-03;
tr = 70E-03;

%% 
mf = bigSportsFigure();
setPose(mf,q);

n0 = 0.75.*ones(1,3);
setColor(mf,n0,n0,n0,n0);
setAlpha(mf,0.2,0,0,0);
delete(axe.Children(1:end - 2));
axis(axe,'equal');

%% Plot road surface

p = 0.3.*[
    -1,-1;
    -1,1;
    1,1;
    1,-1
    ];

hold(axe,'on');
h = fill3(axe,p(:,1),p(:,2),zeros(size(p,1),1),'k');
h.FaceAlpha = 0.25;
uistack(h,'bottom');
hold(axe,'off');

%%

I = eye(3);
p0 = [0,0,0];

% hw = plotFrame(axe,rigidtform3d(rotz(-60),p0), ...
%     'Color',p0);
% 
Ryaw = rotz(q(1));
% hCr = plotFrame(axe,rigidtform3d(Ryaw,[0,0,0]), ...
%     'Color',m.blue);
% 
Rcamber = Ryaw*rotx(q(2));
ptr = tr.*Ryaw(:,3);
% htr = plotFrame(axe,rigidtform3d(Rcamber,ptr), ...
%     'Color',m.orange);
% 
Rpitch = Rcamber*roty(q(3));
prr = ptr + (rr - tr).*Rcamber(:,3);
h = plotFrame(axe,rigidtform3d(Rpitch,prr), ...
    'Color',m.yellow, ...
    'Scale',0.2);

%%

% plotDoFLine(axe, ...
%     rigidtform3d(rotz(-60),p0), ...
%     rigidtform3d(Ryaw,p0), ...
%     'y',0.5.*[1,1]);
% 

plotDoFLine(axe, ...
    rigidtform3d(I,p0), ...
    rigidtform3d(I,ptr), ...
    'y',0.3*[1,1]);

plotDoFLine(axe, ...
    rigidtform3d(I,ptr), ...
    rigidtform3d(Rcamber,ptr), ...
    'z',[-tr,0]);

plotDoFLine(axe, ...
    rigidtform3d(I,ptr), ...
    rigidtform3d(Rcamber,ptr), ...
    'z',-[0,tr/cosd(q(2))]);

plotDoFLine(axe, ...
    rigidtform3d(I,p0), ...
    rigidtform3d(Rcamber,ptr), ...
    'y',[tr*tand(q(2)),0]);

plotDoFLine(axe, ...
    rigidtform3d(I,ptr), ...
    rigidtform3d(Rcamber,ptr), ...
    'z',(2*rr - tr)*[1,1]);

plotDoFLine(axe, ...
    rigidtform3d(Rcamber,ptr), ...
    rigidtform3d(Rcamber,prr), ...
    'y',-0.2*[1,1]);

re = norm(prr);
ang = acosd(([0,0,1]*prr)/(norm([0,0,1])*norm(prr)));

plotDoFLine(axe, ...
    rigidtform3d(I,ptr), ...
    rigidtform3d(rotx(ang).',p0), ...
    'z',re*[0,1]);

plotDoFLine(axe, ...
    rigidtform3d(rotx(ang).',p0), ...
    rigidtform3d(rotx(ang).',[0,0,re]*rotx(ang)), ...
    'y',-0.15*[1,1]);

% plotDoFLine(axe, ...
%     rigidtform3d(I,ptr), ...
%     rigidtform3d(rotx(q(2) + 8),p0), ...
%     'z',(2*rr - tr)*[0,1]);

% 
% plotDoFLine(axe, ...
%     rigidtform3d(Rcamber,prr), ...
%     rigidtform3d(Rpitch,prr), ...
%     'x',0.7.*[1,1]);
% 
% plotDoFLine(axe, ...
%     rigidtform3d(Rcamber,prr), ...
%     rigidtform3d(Rpitch,prr), ...
%     'y',[0,0.5]);
% 
% plotDoFLine(axe, ...
%     rigidtform3d(Ryaw,p0), ...
%     rigidtform3d(Ryaw*rotz(10),p0), ...
%     'x',[0.8,0.7]);

camproj(axe,'perspective');
ylim(axe,[-0.2,0.3]);

%% Save figure
dir = 'C:\Users\marti\PhD\Thesis\MotorcycleDynamics\ForcesAndMoments\Figures\';
saveas(fig,string(dir) + "overturning_moment_unlabeled.eps",'epsc');