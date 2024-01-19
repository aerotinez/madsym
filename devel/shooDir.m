function [T,U,X,gopt] = shooDir()
% Implementation of the direct shooting method
% PARAMETERS:
% pval − number of subintervals (e.g. 20)
% T − array of subsequent time points (i.e., times of shoots)
% U − trajectory of optimal control (discretized)
% X − state trajectory
% gopt − optimal value of objective function
% CALLING:
% >> [T,U,X, gopt]= shooDir(pval)
% For example:
% >> [T,U,X, gopt]= shooDir(20)

global p Dt U

p = 20; 
Dt = 1/p ;

% starting point for (time discretized) trajectory optimization
xs = zeros(p + 1,1);
us = zeros(p,1);
zinit = [xs;xs;xs;us];
U = [];

% initial conditions for state equation
Aeq = zeros(3,4*p + 3);
Aeq(1,1) = 1;
Aeq(2,p + 2) = 1;
Aeq(3,2*p + 3) = 1;
beq = [0;-1;0];

% inequality state constraint discretized in time
A = zeros(p + 1,4*p + 3);
for k = 1:p + 1
    A(k,p + 1 + k) = 1;
    b(k) = 8*((k - 1)*Dt - 0.5)^2 - 0.5;
end

tic
[zo,gopt] = fmincon(@fun,zinit,A,b,Aeq,beq,[],[],@nonlcon,optimset('Display','iter','TolFun',1e-4,'MaxFunEvals',100000));
toc

% The lines below in this function are only for output purposes (including plots)
T = [];
for k = 1:p + 1
    T(k) = Dt*(k - 1);
end

f(1) = figure();
X = [zo(1:p + 1),zo(p + 2:2*(p + 1)),zo(2*p + 3:3*(p + 1))];
h1 = plot(T,X(:,1),'b--',T,X(:,2),'g-',T,8*(T - 0.5).^2 - 0.5, 'm-.');
grid on
for i = 1:3
    h1(i).LineWidth = 3;
end
h1(2).Color = [0,0.6,0.3];
legend('x_1','x_2','S(x, t) = 0');
xlabel('t');
ylabel('x');
txt = {'INFEASIBLE','','x_2 REGION'};
text(0.4,0.8,txt,'Color','m');

f(2) = figure();
plot(T(1:end - 1),U,'LineWidth',3);
grid on;
legend('u optimal - direct method');
xlabel('t');
ylabel('u');
end

function [c,ceq] = nonlcon(z)
    % The function calculates the discrepancies for trajectories on subsequent subintervals
    % between guesses of initial points and the end points of shoots (from the previous guessed points)
    global p Dt U
    xs = zeros(3,p + 1);
    
    for i = 1:3
        for k = 1:p + 1
            xs(i,k) = z((i - 1)*(p + 1) + k);
        end
    end
    disp(xs)
    
    for k = 1:p
        U(k) = z(3*(p + 1) + k);
    end
    
    c = [];
    ceq = [];
    j = 1;
    
    for k = 1:p
        [T,X] = ode45(@(t,x)f(t,x,U(k)),[(k - 1)*Dt,k*Dt],[xs(1,k);xs(2,k);xs(3,k)]);
        for i = 1:3
            ceq(j) = X(end,i) - xs(i,k + 1);
            j = j + 1;
        end
    end
end

function dxdt = f(t,x,u)
    % state transformation function (rhs of the state equation)
    % here x indices denote coordinates
    dxdt = zeros(3,1);
    dxdt(1) = x(2);
    dxdt(2) = -x(2) + u;
    dxdt(3) = x(1)^2 + x(2)^2 + 0.005 * u^2;
end

function fz = fun(z)
    global p
    fz = z(3*(p + 1));
end