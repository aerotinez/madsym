close("all"); clear; clc;
setmadsympath();

%% Parameters
G = 1;
m1 = 1;
m2 = 1;
m3 = 1;

params = [G,m1,m2,m3].';

%% Initial conditions

p1 = [ 0.97000436, -0.24308753, 0]';
p2 = [-0.97000436,  0.24308753, 0]';
p3 = [0, 0, 0]';

v1 = [ 0.4662036850,  0.4323657300, 0]';
v2 = [ 0.4662036850,  0.4323657300, 0]';
v3 = [-0.93240737,   -0.86473146, 0]';

x0 = [
    p1;
    p2;
    p3;
    v1;
    v2;
    v3
    ];

f = @(t,y)threeBodyProblem(y,params);

opts = odeset('MaxStep',0.1);

[t,x] = ode45(f,[0,10],x0,opts);

%% Plot
m = matlabColors;

p1 = x(:,1:3);
p2 = x(:,4:6);
p3 = x(:,7:9);

fig = figure();
axe = axes(fig);
axis(axe,'equal');
view(axe,3);
camproj(axe,'perspective');
box(axe,'on');
xlim(axe,3*[-1,1]);
ylim(axe,3*[-1,1]);
zlim(axe,3*[-1,1]);

hold(axe,'on');
plot3(axe,p1(:,1),p1(:,2),p1(:,3));
plot3(axe,p2(:,1),p2(:,2),p2(:,3));
plot3(axe,p3(:,1),p3(:,2),p3(:,3));
hold(axe,'off');

axis(axe,'tight');

hold(axe,'on');
h1 = scatter3(axe,p1(1,1),p1(1,2),p1(1,3),'filled','CData',m.blue);
h2 = scatter3(axe,p2(1,1),p2(1,2),p2(1,3),'filled','CData',m.orange);
h3 = scatter3(axe,p3(1,1),p3(1,2),p3(1,3),'filled','CData',m.yellow);
hold(axe,'off');

for k = 1:numel(t)
    update(h1,p1(k,:));
    update(h2,p2(k,:));
    update(h3,p3(k,:));
    drawnow;
end

%% Helper functions
function update(h,p)
    h.XData = p(1);
    h.YData = p(2);
    h.ZData = p(3);
end