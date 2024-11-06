close("all"); clear; clc;

ns = 1E03;
slip_max = deg2rad(10);
a0 = 0.01;
slip = linspace(-slip_max,slip_max,ns);

% Lot and Sharp's function
a = a0.*(1 - abs(slip./slip_max));

% cosine approximation
a1 = a0.*cos((pi/2).*(slip./(slip_max))).^2;

fig = figure();
axe = axes(fig);
hold(axe,"on");
plot(axe,rad2deg(slip),a);
plot(axe,rad2deg(slip),a1);
hold(axe,"off");
axis(axe,"square");
box(axe, "on");
title("Tire trail");
xlabel("Slip angle (deg)");
ylabel("Trail (m)");