setmadsympath();
close("all"); clear; clc;

J = @(x)x(1);
x0 = [10;0.1.*ones(7,1)];
A = [];
b = [];
Aeq = [];
beq = [];
lb = [0;-inf(7,1)];
ub = inf(8,1);
opts = optimoptions("fmincon", ...
    "MaxIterations",1e4, ...
    "MaxFunctionEvaluations",1e5, ...
    "OptimalityTolerance",1E-06, ...
    "ConstraintTolerance",1E-04, ...
    "Display","iter-detailed");
sol = fmincon(J,x0,A,b,Aeq,beq,lb,ub,@constraints,opts);
tf = sol(1);
p = sol(2:end);
ode_opts = odeset("MaxStep",1E-03);
y0 = [zeros(4,1);10;zeros(2,1);p];
[t,y] = ode45(@(t,y)travelingInvertedPendulumPMP(y,9.81),[0,tf],y0,ode_opts);
results = [y(:,1:7),slidingMassOptimalControl(y.',1).'];

tiledlayout(4,2);
titles = [
    "x";
    "y";
    "yaw";
    "lean";
    "speed";
    "yaw rate";
    "lean rate"
    ];
for i = 1:7
    nexttile;
    plot(t,results(:,i));
    title(titles(i));
end

function [c,ceq] = constraints(x)
    tf = x(1);
    p = x(2:end);
    y0 = [zeros(4,1);10;zeros(2,1);p];
    ode_opts = odeset("MaxStep",1E-02);
    [~,y] = ode45(@(t,y)travelingInvertedPendulumPMP(y,9.81),[0,tf],y0,ode_opts);
    u = travelingInvertedPendulumOptimalControl(y.',9.81);
    c = [
        max(abs(u),[],2) - [10;100];
        max(abs(y(:,3))) - deg2rad(90);
        max(abs(y(:,4))) - deg2rad(90)
        ];
    yf = [1,0,0,0,10,0,0].';
    ceq = y(end,1:7).' - yf;
end