setmadsympath();
close("all"); clear; clc;

N = 100;

x0 = [
    -ones(N,1);
    1
    ];

A = [];
b = [];
Aeq = [];
beq = [];

lb = [
    -ones(N,1);
    0
    ];

ub = [
    ones(N,1);
    inf
    ];


options = optimoptions("fmincon","MaxFunEvals",1E05,"Display","iter-detailed");
sol = fmincon(@fun,x0,A,b,Aeq,beq,lb,ub,@nonlcon,options);
u = sol(1:end-1);
tf = sol(end);
optimalcontrol = @(t)interp1(linspace(0,tf,N),u,t);
sys = @(t,x)(1/tf).*plant(x,optimalcontrol(t));
[t,y] = ode45(sys,[0,tf],[0;0;tf],odeset("MaxStep",1E-03));

results = [
    y(:,1:2).';
    optimalcontrol(t).'
    ];

tiledlayout(size(results,1),1);
for i = 1:size(results,1)
    nexttile;
    plot(t,results(i,:));
    xlim([0,tf]);
    ylim(1.5.*max(results(i,:)).*[-1,1])
end

function x_dot = plant(x,u)
    x_dot = x(end)*[
        slidingMass(x(1:end-1),u,1);
        0
    ];
end

function J = fun(x)
    u = x(1:end-1);
    tf = x(end);
    J = tf + (1/2).*u.'*u;
    % J = tf;
end

function y_dot = rk4(y,u,h)
    k1 = plant(y,u);
    k2 = plant(y + (h/2).*k1,u);
    k3 = plant(y + (h/2).*k2,u);
    k4 = plant(y + h.*k3,u);
    y_dot = y + (h/6).*(k1 + 2.*k2 + 2.*k3 + k4);
end

function [c,ceq] = nonlcon(x)
    c = [];
    u = x(1:end-1);
    tf = x(end);
    N = numel(u);
    y0 = [0;0;tf];
    y = [y0,zeros(3,N-1)];
    h = 1/N;
    for i = 1:N-1
        y(:,i+1) = rk4(y(:,i),u(i),h); 
    end
    yf = [1;0];
    ceq = y(1:end-1,end) - yf;
end