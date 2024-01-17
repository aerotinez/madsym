setmadsympath();
close("all"); clear; clc;

J = @(x)x(1);
x0 = [1;0;0];
A = [];
b = [];
Aeq = [];
beq = [];
lb = [0;-inf;-inf];
ub = inf(3,1);
opts = optimset("TolCon",1E-04,"Display","iter-detailed");
sol = fmincon(J,x0,A,b,Aeq,beq,lb,ub,@slidingMassConstraints,opts);
tf = sol(1);
p = sol(2:end);
ode_opts = odeset("MaxStep",1E-03);
[t,y] = ode45(@(t,y)slidingMassPMP(y,1),[0,tf],[0;0;p],ode_opts);
results = [y(:,1:2),slidingMassOptimalControl(y.',1).'];

tiledlayout(3,1);
for i = 1:3
    nexttile;
    plot(t,results(:,i));
end

function [c,ceq] = slidingMassConstraints(x)
    tf = x(1);
    p = x(2:end);
    y0 = [0;0;p];
    [~,y] = ode45(@(t,y)slidingMassPMP(y,1),[0,tf],y0);
    u = slidingMassOptimalControl(y.',1);
    c = max(abs(u)) - 10;
    yf = [1,0].';
    ceq = y(end,1:2).' - yf;
end