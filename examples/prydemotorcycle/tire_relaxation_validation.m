close("all"); clear; clc;
setmadsympath();

%% BikeSim results
vx = [30,50,80,110,130];
plant = @prydeMotorcycleLateralStateSpace;
n2s = @num2str;
results_path = "G:\My Drive\BikeSimResults\BigSports\Chicane";

k = 2;
speed_path = "\Vx" + n2s(vx(k)) + "Kph\";
file_name = "bikesim_results_" + n2s(vx(k)) + "kph.csv";
results = readtable(results_path + speed_path + file_name);

time = results.Time;
ts = 1/60;

s = 5;
af = results.Alpha_1;
alf = results.AlphL_1;
vxf = results.VxWC_1./3.6;
vyf = results.VyCGC_1./3.6;

ar = results.Alpha_2;
alr = results.AlphL_2;
vxr = results.VxWC_2./3.6;
vyr = results.VyCGC_2./3.6;

a = [af,ar];
al = [alf,alr];

af_est = atan2d(vyf,vxf);
ar_est = atan2d(vyr,vxr);

a_est = [af_est,ar_est];

f = @(vx,vy,a)(1/s)*cosd(a).*(cosd(a).*vy - sind(a).*vx);
g = @(t,x)interp1(time,x,t,'spline');
h = @(t,y,vx,vy,a)f(g(t,vx),g(t,vy),y);

dalf_est = f(vxf,vyf,alf);
[~,alf_est] = ode45(@(t,y)h(t,rad2deg(y),vxf,vyf),time,deg2rad(alf(1)));
alf_est = rad2deg(alf_est);

dalr_est = f(vxr,vyr,alr);
[~,alr_est] = ode45(@(t,y)h(t,rad2deg(y),vxr,vyr),time,deg2rad(alr(1)));
alr_est = rad2deg(alr_est);

al_est = [alf_est,alr_est];

fig = figure();
tl = tiledlayout(2,1);

titles = [
    "Front";
    "Rear"
    ];

for i = 1:2
    axe = nexttile(i);
    hold(axe,'on');
    plot(time,a(:,i),'LineWidth',2);
    plot(time,al(:,i),'LineWidth',2);
    plot(time,a_est(:,i),'--','LineWidth',2);
    plot(time,al_est(:,i),'--','LineWidth',2);
    hold(axe,'off');
    box(axe,'on');
    axis(axe,'tight');
    title(axe,titles(i));
    ylabel(axe,"slip (deg)");
end
xlabel(axe,'time (s)');
sgtitle(fig,"Slip vs lagged slip");
leg = legend("slip","lagged slip","est slip","est lagged slip");
leg.Orientation = "horizontal";
leg.Layout.Tile = 'south';
