close("all"); clear; clc;
setmadsympath();

m = matlabColors();

B = 4;
C = 2.2;
D = 1000;
E = 0.99;

ns = 1E03;
x = linspace(0,1,ns);
y = D*sin(C*atan(B*x - E*(B*x - atan(B*x))));
t = atan(B*C*D);
disp(rad2deg(t))

R = [
    cos(t),-sin(t);
    sin(t),cos(t)
    ];

p0 = [
    0,900;
    0,0
    ];

p = R*p0;

fig = figure('Position',[100,100,640,240]);
axe = axes(fig);

hold(axe,'on');

h = plot(axe,x,y,'LineWidth',1);
hC = plot(axe,[0.7,1],y(end).*[1,1],'k--','LineWidth',1);
hD = plot(axe,[-0.1,0.5],max(y).*[1,1],'k--','LineWidth',1);
ht = plot(axe,p(1,:),p(2,:),'k--','LineWidth',1);

hold(axe,'off');

box(axe,'on');
xlim(axe,[-0.15,1]);
ylim(axe,[0,1100]);

xlabel(axe,'\chi', ...
    'FontSize',12, ...
    'Interpreter','tex');

ylabel(axe,'f(\chi) (N)', ...
    'FontSize',12, ...
    'Interpreter','tex');

title(axe,"Pacejka's magic formula");

[xMaxFig,y0Fig] = ds2nfu(axe,-0.025,0);
[~,yMaxFig] = ds2nfu(axe,-0.025,D);
annotation('doublearrow',[xMaxFig,xMaxFig],[y0Fig,yMaxFig],...
           'LineWidth',1);

text(-0.075,500,'$D$', ...
    'FontSize',12, ...
    'Interpreter','latex');

[xMaxFig,y0Fig] = ds2nfu(axe,0.9,0);
[~,yMaxFig] = ds2nfu(axe,0.9,y(end));
annotation('doublearrow',[xMaxFig,xMaxFig],[y0Fig,yMaxFig],...
           'LineWidth',1);

text(0.7,450,'$D\sin\bigl(\frac{\pi}{2}C\bigr)$', ...
    'FontSize',12, ...
    'Interpreter','latex');

ns = 100;
ang = linspace(0,t - 13*t/ns,100);
xArc = 0.06*cos(ang);
yArc = 100*sin(ang);

text(0.05,100,'$\arctan(BCD)$', ...
    'FontSize',12, ...
    'Interpreter','latex');

hold(axe,'on');
plot(axe, xArc, yArc, 'k-','LineWidth',1.5);
hold(axe,'off');

dir = "C:\Users\marti\PhD\Thesis\MotorcycleDynamics\ForcesAndMoments\Figures\";
saveas(fig,dir + "magic_formula.eps",'epsc');