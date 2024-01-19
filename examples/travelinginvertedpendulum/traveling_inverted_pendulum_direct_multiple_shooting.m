setmadsympath();
close("all"); clear; clc;

N = 25;

x0 = zeros(N,1);
y0 = zeros(N,1);
yaw0 = zeros(N,1);
lean0 = zeros(N,1);
v0 = 20.*ones(N,1);
yaw_rate0 = zeros(N,1);
lean_rate0 = zeros(N,1);
Fx0 = zeros(N,1);
Mz0 = zeros(N,1);

x0 = [
    x0;
    y0;
    yaw0;
    lean0;
    v0;
    yaw_rate0;
    lean_rate0;
    Fx0;
    Mz0;
    1
    ];

A = [];
b = [];
Aeq = [];
beq = [];

xlb = -inf(N,1);
ylb = -inf(N,1);
yawlb = -deg2rad(90).*ones(N,1);
leanlb = -deg2rad(90).*ones(N,1);
vlb = -inf(N,1);
yaw_ratelb = -inf(N,1);
lean_ratelb = -inf(N,1);
Fxlb = -ones(N,1);
Mzlb = -10.*ones(N,1);
lb = [
    xlb;
    ylb;
    yawlb;
    leanlb;
    vlb;
    yaw_ratelb;
    lean_ratelb;
    Fxlb;
    Mzlb;
    0
    ];

xub = inf(N,1);
yub = inf(N,1);
yawub = deg2rad(90).*ones(N,1);
leanub = deg2rad(90).*ones(N,1);
vub = inf(N,1);
yaw_rateub = inf(N,1);
lean_rateub = inf(N,1);
Fxub = ones(N,1);
Mzub = 10.*ones(N,1);
ub = [
    xub;
    yub;
    yawub;
    leanub;
    vub;
    yaw_rateub;
    lean_rateub;
    Fxub;
    Mzub;
    inf
    ];


options = optimoptions("fmincon", ...
    "MaxFunEvals",5E04, ...
    "Display","iter-detailed", ...
    "UseParallel",true);

sol = fmincon(@fun,x0,A,b,Aeq,beq,lb,ub,@nonlcon,options);
tf = sol(end);
X = reshape(sol(1:end-1),N,[]);
Fx = X(:,end - 1);
Mz = X(:,end);

optimalcontrol = @(t)interp1(linspace(0,tf,N),[Fx,Mz],t);
sys = @(t,x)(1/tf).*plant(x,optimalcontrol(t).');
[t,y] = ode45(sys,[0,tf],[0;0;0;0;0;0;0;tf]);

results = [
    y(:,1:7).';
    optimalcontrol(t).'
    ];

titles = [
    "x";
    "y";
    "yaw";
    "lean";
    "speed";
    "yaw rate";
    "lean rate";
    "F_x";
    "M_z"
    ];

tiledlayout(3,3);
for i = 1:size(results,1)
    nexttile;
    plot(t,results(i,:));
    xlim([0,tf]);
    title(titles(i));
    % ylim(1.5.*max(results(i,:)).*[-1,1])
end

function x_dot = plant(x,u)
    x_dot = x(end).*[
        travelingInvertedPendulum(x(1:end-1),u,[1,0,0.1,9.81,1,1].');
        0
        ];
end

function J = fun(x)
    tf = x(end);
    X = reshape(x(1:end-1),[],9);
    x = reshape(X(:,1:7),[],1);
    u = reshape(X(:,end-1:end),[],1);
    J = tf;
end

function [c,ceq] = nonlcon(x)
    % no nonlinear inequality constraints
    c = [];

    % unpack decision variables
    tf = x(end);
    X = reshape(x(1:end-1),[],9);
    x0 = X(:,1);
    y0 = X(:,2);
    yaw0 = X(:,3);
    lean0 = X(:,4);
    v0 = X(:,5);
    yaw_rate0 = X(:,6);
    lean_rate0 = X(:,7);
    Fx = X(:,8);
    Mz = X(:,9);

    % recover mesh resolution
    N = numel(Fx);

    % initial conditions for each segment
    y0 = [
        x0.';
        y0.';
        yaw0.';
        lean0.';
        v0.';
        yaw_rate0.';
        lean_rate0.'
        ];

    % for each time span and initial condition, simulate the system and store
    % the end state
    yf = zeros(7,N);
    for i = 1:N-1
        % simulate the system
        [~,y] = ode45(@(t,x)plant(x,[Fx(i);Mz(i)]),[0,1/N],[y0(:,i);tf]);
        yf(:,i+1) = y(end,1:7).';
    end
    x0 = [0;0;0;0;0;0;0];
    xf = [1;1;0;0;0;0;0];
    ceq = reshape([y0(:,1) - x0,y0(:,2:end) - yf(:,2:end),y0(:,end) - xf],[],1);
end