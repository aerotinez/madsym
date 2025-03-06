close("all"); clear; clc;
setmadsympath();

m = matlabColors();

B = 10;
C = 2.1;
D = 1E03;
E = 0.99;

ns = 1E03;
x = linspace(0,1,ns);

f = @(B,C,D,E,x)D*sin(C*atan(B*x - E*(B*x - atan(B*x))));

dB = 5;
yBmin = f(B - dB,C,D,E,x);
yBmax = f(B + dB,C,D,E,x);

dC = 0.2;
yCmin = f(B,C - dC,D,E,x);
yCmax = f(B,C + dC,D,E,x);

dD = 50;
yDmin = f(B,C,D - dD,E,x);
yDmax = f(B,C,D + dD,E,x);

dE = 0.01;
yEmin = f(B,C,D,E - dE,x);
yEmax = f(B,C,D,E - 2*dE,x);

fig = figure('Position',[100,100,640,360]);
axe = axes(fig);
hold(axe,'on');

% l = plot(axe,x,y, ...
%     "Color",m.blue, ...
%     "LineWidth",1);

hB = patch(axe,[x,fliplr(x)].',[yBmin,fliplr(yBmax)].','k', ...
    'FaceColor',m.blue, ...
    'EdgeColor','none', ...
    'FaceAlpha',0.5);

hC = patch(axe,[x,fliplr(x)].',[yCmin,fliplr(yCmax)].','k', ...
    'FaceColor',m.orange, ...
    'EdgeColor','none', ...
    'FaceAlpha',0.5);

hD = patch(axe,[x,fliplr(x)].',[yDmin,fliplr(yDmax)].','k', ...
    'FaceColor',m.yellow, ...
    'EdgeColor','none', ...
    'FaceAlpha',0.5);

hE = patch(axe,[x,fliplr(x)].',[yEmin,fliplr(yEmax)].','k', ...
    'FaceColor',m.purple, ...
    'EdgeColor','none', ...
    'FaceAlpha',0.5);

hold(axe,'off');
box(axe,'on');
xlim(axe,[-0.2,1]);
ylim(axe,[0,1200]);
