setmadsympath();
close("all"); clear; clc;

N = 20;

p0 = zeros(N,1);
v0 = zeros(N,1);
u = ones(N,1);

x0 = [
    p0;
    v0;
    u;
    1
    ];

A = [];
b = [];
Aeq = [];
beq = [];

plb = -inf(N,1);
vlb = -inf(N,1);
ulb = -ones(N,1);
lb = [
    plb;
    vlb;
    ulb;
    0
    ];

pub = inf(N,1);
vub = inf(N,1);
uub = ones(N,1);
ub = [
    pub;
    vub;
    uub;
    inf
    ];


options = optimoptions("fmincon", ...
    "MaxFunEvals",1E05, ...
    "Display","iter-detailed", ...
    "UseParallel",true);

sol = fmincon(@fun,x0,A,b,Aeq,beq,lb,ub,@nonlcon,options);
tf = sol(end);
X = reshape(sol(1:end-1),[],3);
u = X(:,3);

optimalcontrol = @(t)interp1(linspace(0,tf,N),u,t);
sys = @(t,x)(1/tf).*plant(x,optimalcontrol(t));
[t,y] = ode45(sys,[0,tf],[0;0;tf]);

results = [
    y(:,1:2).';
    optimalcontrol(t).'
    ];

tiledlayout(size(results,1),1);
for i = 1:size(results,1)
    nexttile;
    plot(t,results(i,:));
    xlim([0,tf]);
    % ylim(1.5.*max(results(i,:)).*[-1,1])
end

function x_dot = plant(x,u)
    x_dot = x(3).*[
        slidingMass(x(1:2),u,1);
        0
        ];
end

function J = fun(x)
    X = reshape(x(1:end-1),[],3);
    u = X(:,3);
    tf = x(end);
    J = tf;
end

function [c,ceq] = nonlcon(x)
    % no nonlinear inequality constraints
    c = [];

    % unpack decision variables
    tf = x(end);
    X = reshape(x(1:end-1),[],3);
    p0 = X(:,1);
    v0 = X(:,2);
    u = X(:,3);

    % recover mesh resolution
    N = numel(u);

    % initial conditions for each segment
    y0 = [
        p0.';
        v0.'
        ];

    % for each time span and initial condition, simulate the system and store
    % the end state
    yf = zeros(2,N);
    for i = 1:N
        % simulate the system
        [~,y] = ode45(@(t,x)plant(x,u(i)),[0,1/N],[y0(:,i);tf]);
        yf(:,i) = y(end,1:2).';
    end
    x0 = [0;0];
    xf = [1;0];
    ceq = reshape([y0,xf] - [x0,yf],[],1);
end