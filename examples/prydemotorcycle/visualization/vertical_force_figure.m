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

lwb = 1300E-03;


%% Plot and format motorcycle
q = [0,0,0,0];

mf = bigSportsFigure();
setPose(mf,q);
n = 0.5.*[1,1,1];
setColor(mf,n,n,n,n);
setAlpha(mf,0.25,0.25,0.25,0.25);

%% Mass centers

R = [
    cosd(caster),-sind(caster);
    sind(caster),cosd(caster)
    ];

hold(axe,'on');

% Ground
plot3(axe,[0,lwb],[0,0],[0,0],'k','LineWidth',3);

plotPoint([lwb,rf]);
plotPoint([b,0] + [a + lz + e,f]*R.');
plotPoint([b,h]);
plotPoint([0,rr]);

hold(axe,"off");

%% Save figure
dir = 'C:\Users\marti\PhD\Thesis\MotorcycleDynamics\ForcesAndMoments\Figures\';
saveas(fig,string(dir) + "vertical_forces_unlabled.eps",'epsc');

%% Helper functions
function plotPoint(p)
    arguments
        p (1,2) double;
    end
    axe = gca;
    hold(axe,"on");
    h = scatter3(axe,p(1),0,p(2),20,'white','filled', ...
        'LineWidth',2, ...
        'MarkerEdgeColor','k');
    hold(axe,"off");
    uistack(h,"top");
end