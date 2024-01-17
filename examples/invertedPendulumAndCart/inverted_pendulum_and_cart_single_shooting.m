setmadsympath();
close("all"); clear; clc;

J = @(x)x(1);
x0 = [10;-ones(4,1)];
A = [];
b = [];
Aeq = [];
beq = [];
lb = [0;-inf(4,1)];
ub = inf(5,1);
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
y0 = [zeros(4,1);p];
params = [0.006,0.1,9.81,0.3,0.5,0.2].';
[t,y] = ode45(@(t,y)invertedPendulumAndCartPMP(y,params),[0,tf],y0,ode_opts);
results = [y(:,1:4),invertedPendulumAndCartOptimalControl(y.',params).'];

tiledlayout(2,2);
titles = [
    "x";
    "theta";
    "v";
    "theta dot"
    ];
for i = 1:4
    nexttile;
    plot(t,results(:,i));
    title(titles(i));
end

function [c,ceq] = constraints(x)
    tf = x(1);
    p = x(2:end);
    y0 = [zeros(4,1);p];
    ode_opts = odeset("MaxStep",1E-03);
    params = [0.006,0.1,9.81,0.3,0.5,0.2].';
    [~,y] = ode45(@(t,y)invertedPendulumAndCartPMP(y,params),[0,tf],y0,ode_opts);
    u = invertedPendulumAndCartOptimalControl(y.',params);
    c = max(abs(u),[],2) - 100;
    yf = [1,0,0,0].';
    ceq = y(end,1:4).' - yf;
end