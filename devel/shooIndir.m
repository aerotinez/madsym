function [T,U,Y,gopt] = shooIndir
%   Implementation of the indirect shooting method
%   PARAMETERS:
%   T - arrayof subsequent time points of controlinterval
%   U - trajectory of optimal control (discretized)
%   Y - state and adjoint variables trajectories (discretized)
%   gopt - optimalvalue of objectivefunction
%   CALLING: >> [T,U,Y,gopt] = shooIndir

global TTot YTot

zs = [0;0;0.33;0.66;1];

tic
[zo,~] = fsolve(@ShooF,zs,optimset('Display','iter'));
T = TTot;
Y = YTot;
toc

% The lines below in this function are only for output purposes (including 
% plots)
stT = size(T);
U = zeros(size(T));
MuS = zeros(size(T));
for t = 1:stT
    if T(t) < zo(3) || T(t) > zo(4)
        U(t)= 100*Y(t,5);
        MuS(t) = 0.;
    else
        U(t) = Y(t,2) + 16*(T(t) - 0.5);
        MuS(t) = Y(t,5) - 0.01*U(t);
    end
end
gopt = Y(end,3);

f(1) = figure();
h1 = plot(T,Y(:,1),'b--',T,Y(:,2),'g-',T,8*(T - 0.5).^2 - 0.5,'m-.');
for i=1:3
    h1(i).LineWidth = 3;
end
h1(2).Color = [0,0.6,0.3];
grid on;
legend('x_1 ','x_2 ','S(x,t)=0');
xlabel('t');
ylabel('x');
txt = {'INFEASIBLE','',' x_2 REGION'};
text(0.4,0.8,txt,'Color','m');

f(2) = figure();
h2 = plot(T,Y(:,4),'r-',T,Y(:,5),'b--',T,MuS','g-.');
h2(3).Color = [0,0.6,0.3];
grid on;
legend('\eta_1','\eta_2','\mu_S');
xlabel('t');
ylabel('\eta,\mu_S ');
for i=1:3
    h2(i).LineWidth=3;
end

f(3) = figure();
plot(T,U,'LineWidth',3);
grid on;
legend('u optimal-indirect method');
xlabel('t');
ylabel('u');
end

function Ferr = ShooF(z)
% The function calculates lhs of the ultimate set of (static) nonlinear 
% equations: Ferr(z) = 0
global TTot YTot

eT10 = z(1);
eT20 = z(2);
t1 = z(3);
t2 = z(4);
gamma = z(5);
TTot = [];
YTot = [];

% Region1 (state constraint inactive)
[T,Y] = ode45(@(t,y)f(t,y,1),[0,t1],[0;-1;0;eT10;eT20]);
TTot = T;
YTot = Y;
x11 = Y(end,1);
x21 = Y(end,2);
x31 = Y(end,3);
eT11 = Y(end,4);
eT21 = Y(end,5);
x2b1 = 8*(t1 - 0.5)^2-0.5;
Ferr(1) = x21 - x2b1;

% Region 2  (state constraint active)
[T,Y] = ode45(@(t,y)f(t,y,2),[t1,t2],[x11;x21;x31;eT11;eT21 + gamma]);
TTot = [TTot;T]; 
YTot = [YTot;Y];
x12 = Y(end,1);
x22 = Y(end,2);
x32 = Y(end,3);
eT12 = Y(end,4);
eT22 = Y(end,5);

%  Region 3  (state constraint inactive)
[T,Y] = ode45(@(t,y)f(t,y,3),[t2,1],[x12;x22;x32;eT12;eT22]);
TTot = [TTot;T]; 
YTot = [YTot;Y];
Ferr(2) = Y(end,4);
Ferr(3) = Y(end,5);
end

function dydt = f(t,y,region)
% extended state transformation function (rhs of the 1st order ODE in state
% and adjoint variables space);
% here : y = [x1,x2,x3,eta1,eta2]
dydt = zeros(5,1);
dydt(1) = y(2);
switch region
    case {1,3}
        u = 100*y(5);
        muS = 0;
    case 2
        u = y(2) + 16*(t - 0.5);
        muS = y(5) - 0.01*u;
end
dydt(2) = -y(2) + u;
dydt(3) = y(1)^2 + y(2)^2 + 0.005*u^2;
dydt(4) = 2*y(1);
dydt(5) = 2*y(2) - y(4) + y(5) - muS;
end