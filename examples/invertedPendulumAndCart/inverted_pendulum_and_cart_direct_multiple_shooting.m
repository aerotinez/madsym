setmadsympath();
close("all"); clear; clc;

N = 50;

p0 = zeros(N,1);
theta0 = zeros(N,1);
v0 = zeros(N,1);
w0 = zeros(N,1);
u = ones(N,1);

x0 = [
    p0;
    theta0;
    v0;
    w0;
    u;
    1
    ];

A = [];
b = [];
Aeq = [];
beq = [];

plb = -inf(N,1);
thetalb = -deg2rad(180).*ones(N,1);
vlb = -inf(N,1);
wlb = -inf(N,1);
ulb = -ones(N,1);
lb = [
    plb;
    thetalb;
    vlb;
    wlb;
    ulb;
    0
    ];

pub = inf(N,1);
thetaub = deg2rad(180).*ones(N,1);
vub = inf(N,1);
wub = inf(N,1);
uub = ones(N,1);
ub = [
    pub;
    thetaub;
    vub;
    wub;
    uub;
    inf
    ];


options = optimoptions("fmincon", ...
    "MaxFunEvals",2E05, ...
    "Display","iter-detailed", ...
    "UseParallel",true);

sol = fmincon(@fun,x0,A,b,Aeq,beq,lb,ub,@nonlcon,options);
tf = sol(end);
X = reshape(sol(1:end-1),[],5);
u = X(:,end);

optimalcontrol = @(t)interp1(linspace(0,tf,N),u,t);
sys = @(t,x)(1/tf).*plant(x,optimalcontrol(t));
[t,y] = ode45(sys,[0,tf],[0;0;0;0;tf]);

results = [
    y(:,1:4).';
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
    x_dot = x(end).*[
        invertedPendulumAndCart(x(1:end-1),u,[-9.81,0.3,0.5,0.2].');
        0
        ];
end

function J = fun(x)
    X = reshape(x(1:end-1),[],5);
    u = X(:,end);
    tf = x(end);
    J = tf;
end

function [c,ceq] = nonlcon(x)
    % no nonlinear inequality constraints
    c = [];

    % unpack decision variables
    tf = x(end);
    X = reshape(x(1:end-1),[],5);
    p0 = X(:,1);
    theta0 = X(:,2);
    v0 = X(:,3);
    w0 = X(:,4);
    u = X(:,5);

    % recover mesh resolution
    N = numel(u);

    % initial conditions for each segment
    y0 = [
        p0.';
        theta0.';
        v0.';
        w0.'
        ];

    % for each time span and initial condition, simulate the system and store
    % the end state
    yf = zeros(4,N);
    for i = 1:N
        % simulate the system
        [~,y] = ode45(@(t,x)plant(x,u(i)),[0,1/N],[y0(:,i);tf]);
        yf(:,i) = y(end,1:4).';
    end
    x0 = [0;0;0;0];
    xf = [1;0;0;0];
    ceq = reshape([y0,xf] - [x0,yf],[],1);
end