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

y = [
    f(B - 0.1*B,C,D,E,x),fliplr(f(B + 0.1*B,C,D,E,x));
    f(B,C - 0.1*C,D,E,x),fliplr(f(B,C + 0.1*C,D,E,x));
    f(B,C,D - 0.1*D,E,x),fliplr(f(B,C,D + 0.1*D,E,x));
    f(B,C,D,E - 0.2*E,x),fliplr(f(B,C,D,E,x));
    ];

fig = figure('Position',[100,100,640,360]);
tl = tiledlayout(2,2,'Parent',fig);

for k = 1:4
    axe = nexttile(tl,k);
    hold(axe,'on');
    hB = patch(axe,[x,fliplr(x)].',y(k,:).','k', ...
        'FaceColor',m.blue, ...
        'EdgeColor','none');
    hold(axe,'off');
    box(axe,'on');
    axis(axe,'tight');
end
