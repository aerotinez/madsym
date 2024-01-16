setmadsympath();
close("all"); clear; clc;

J = @(x)x(1);
ns = 33;
x0 = [1;ones(ns,1)];
A = [];
b = [];
Aeq = [];
beq = [];
lb = [0;-1.*ones(ns,1)];
ub = [inf;1.*ones(ns,1)];
options = optimset("TolFun",1E-04,"MaxIter",1E05,"MaxFunEvals",1E04);
y = fmincon(J,x0,A,b,Aeq,beq,lb,ub,@slidingMassConstraints,options);

function [c,ceq] = slidingMassConstraints(x)
    c = [];
    T = x(1);
    ts = T/(numel(x) - 1);
    tsim = 0:ts:(T - ts);
    tspan = [0,T - ts];
    u = @(t)interp1(tsim,x(2:end),t);
    options = odeset("MaxStep",1E-02);
    plant = @(t,y)slidingMass(y,u(t),1);
    [~,y] = ode45(plant,tspan,[0;0],options);
    ceq = y(end,:) - [1,0];
end