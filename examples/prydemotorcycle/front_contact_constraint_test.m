close('all'); clear; clc;
setmadsympath();

%% BikeSim results
speed = 130;
results_path = "G:\My Drive\BikeSimResults\BigSports\Chicane";

n2s = @num2str;
speed_path = "\Vx" + n2s(speed) + "Kph\";
file_name = "bikesim_results_" + n2s(speed) + "kph.csv";
results = readtable(results_path + speed_path + file_name);

params = bikeSimToPrydeParameters(bigSportsParameters,speed/3.6);

steer = deg2rad(results.Steer);
steer_rate = deg2rad(-results.M_StrSys./0.2212);

yaw = deg2rad(results.Yaw);
pitch = deg2rad(results.Pitch);
roll = deg2rad(results.Roll_E);

wx = deg2rad(results.AVx);
wy = deg2rad(results.AVy);
wz = deg2rad(results.AVz);

R = angle2dcm(yaw,pitch,roll);
[yaw,camber,pitch] = dcm2angle(R,"ZXY");

a = params.a;
b = params.b;
rf = params.rf;
tf = params.tf;
rr = params.rr;
tr = params.tr;
caster = params.caster;

c = cos(caster);
s = sin(caster);

Rf = rf + tf;
lx = Rf*s - params.an;
lz = -(Rf*c^2 - a*s + params.an*s)/c;

p = [
    a;
    b;
    lx;
    lz;
    rf;
    tf;
    rr;
    tr;
    caster
    ].';

q = [yaw,camber,pitch,steer];
u = [wx,wy,wz];

est = frontContactConstraint(q,u,p);

% f = @frontContactConstraint;
% 
% % Preallocate output
% est = zeros(size(camber));
% 
% opts = optimoptions('lsqnonlin', ...
%     'Display', 'off', ...
%     'FunctionTolerance', 1e-10, ...
%     'StepTolerance', 1e-10, ...
%     'MaxIterations', 100, ...
%     'SpecifyObjectiveGradient', true);
% 
% for i = 1:length(camber) - 1
%     % Combine residual and Jacobian into one function using `deal`
%     resid_fun = @(x) deal( ...
%         frontContactConstraint(x, [camber(i), pitch(i)], p), ...
%         frontContactConstraintJac(x, [camber(i), pitch(i)], p) );
% 
%     % Solve with lsqnonlin
%     est(i + 1) = lsqnonlin(resid_fun, est(i), -pi, pi, opts);
% end

hold on;
plot(steer_rate(end - 100:end));
plot(est(end - 100:end));
hold off;
axis tight;
box on;
legend('ref','est');