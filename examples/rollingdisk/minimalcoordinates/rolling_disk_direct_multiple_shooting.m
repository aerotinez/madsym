setmadsympath();
close("all"); clear; clc;

N = 50;
Mymax = 10;
Mzmax = 30;

x0 = zeros(N,1);
y0 = zeros(N,1);
yaw0 = zeros(N,1);
lean0 = zeros(N,1);
pitch0 = zeros(N,1);
yaw_rate0 = zeros(N,1);
lean_rate0 = zeros(N,1);
pitch_rate0 = zeros(N,1);
My0 = zeros(N,1);
Mz0 = zeros(N,1);

x0 = [
    x0;
    y0;
    yaw0;
    lean0;
    pitch0;
    yaw_rate0;
    lean_rate0;
    pitch_rate0
    My0;
    Mz0;
    1
    ];

A = [];
b = [];
Aeq = [];
beq = [];

xlb = -inf(N,1);
ylb = -inf(N,1);
yawlb = -deg2rad(45).*ones(N,1);
leanlb = -deg2rad(60).*ones(N,1);
pitchlb = -inf(N,1);
yaw_ratelb = -inf(N,1);
lean_ratelb = -inf(N,1);
pitch_ratelb = -inf(N,1);
Mylb = -Mymax.*ones(N,1);
Mzlb = -Mzmax.*ones(N,1);
lb = [
    xlb;
    ylb;
    yawlb;
    leanlb;
    pitchlb;
    yaw_ratelb;
    lean_ratelb;
    pitch_ratelb;
    Mylb;
    Mzlb;
    0
    ];

xub = inf(N,1);
yub = inf(N,1);
yawub = deg2rad(45).*ones(N,1);
leanub = deg2rad(60).*ones(N,1);
pitchub = inf(N,1);
yaw_rateub = inf(N,1);
lean_rateub = inf(N,1);
pitch_rateub = inf(N,1);
Myub = Mymax.*ones(N,1);
Mzub = Mzmax.*ones(N,1);
ub = [
    xub;
    yub;
    yawub;
    leanub;
    pitchub;
    yaw_rateub;
    lean_rateub;
    pitch_rateub;
    Myub;
    Mzub;
    inf
    ];


options = optimoptions("fmincon", ...
    "MaxFunEvals",1E07, ...
    "Display","iter-detailed", ...
    "UseParallel",true);

sol = fmincon(@fun,x0,A,b,Aeq,beq,lb,ub,@nonlcon,options);
tf = sol(end);
X = reshape(sol(1:end-1),N,[]);

results = X.';

titles = [
    "x";
    "y";
    "yaw";
    "lean";
    "pitch";
    "yaw rate";
    "lean rate";
    "pitch rate";
    "M_y";
    "M_z"
    ];

t = linspace(0,tf,N);
tiledlayout(5,2);
for i = 1:size(results,1)
    nexttile;
    plot(t,results(i,:));
    xlim([0,tf]);
    title(titles(i));
    % ylim(1.5.*max(results(i,:)).*[-1,1])
end

function x_dot = plant(x,u)
    N = size(u,2);
    X = reshape(x,[],N);
    X_dot = zeros(size(X));
    for i = 1:N
        X_dot(:,i) = X(end,i).*[
            rollingDisk(X(1:end-1,i),u(:,i),[9.81,1,0.15].');
            0
            ];
    end
    x_dot = reshape(X_dot,[],1);
end

function J = fun(x)
    tf = x(end);
%     X = reshape(x(1:end-1),[],10);
%     x = reshape(X(:,1:7),[],1);
%     u = reshape(X(:,end-1:end),[],1);
    J = tf;
end

function [c,ceq] = nonlcon(x)
    % no nonlinear inequality constraints
    c = [];

    % unpack decision variables
    tf = x(end);
    X = reshape(x(1:end-1),[],10);
    x0 = X(:,1);
    y0 = X(:,2);
    yaw0 = X(:,3);
    lean0 = X(:,4);
    pitch0 = X(:,5);
    yaw_rate0 = X(:,6);
    lean_rate0 = X(:,7);
    pitch_rate0 = X(:,8);
    My = X(:,9);
    Mz = X(:,10);

    % recover mesh resolution
    N = numel(My);

    % initial conditions for each segment
    y0 = [
        x0.';
        y0.';
        yaw0.';
        lean0.';
        pitch0.';
        yaw_rate0.';
        lean_rate0.';
        pitch_rate0.';
        tf + 0.*x0.'
        ];

    % for each time span and initial condition, simulate the system and store
    % the end state
    [~,y] = ode45(@(t,x)plant(x,[My,Mz].'),[0,1/N],reshape(y0,[],1));
    yf = reshape(y(end,:).',[],N);
    idx = [1:4,6:8];
    x0 = [0;0;0;0;0;0;0];
    xf = [1;0.1;0;0;0;0;0];
    ceq = reshape([y0(idx,:),xf] - [x0,yf(idx,:)],[],1);
end