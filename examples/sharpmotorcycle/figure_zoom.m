close all; clear; clc;
fig = figure();
axe_1 = axes(fig);

t = linspace(-pi,pi,100);
w = 2;
f1 = sin(w.*t);
A = 1 + 1E-02;
phi = deg2rad(5);
f2 = A.*sin(w.*t + phi);

hold(axe_1,"on");
l11 = line(t,f1,"Parent",axe_1,"Color",'r');
l21 = line(t,f2,"Parent",axe_1,"Color",'b');
hold(axe_1,"off");
box(axe_1,"on");
grid(axe_1,"on");

axe_2 = axes("position",[0.2,0.2,0.18,0.18]);
l12 = line(t,f1,"Parent",axe_2,"Color",'r');
l22 = line(t,f2,"Parent",axe_2,"Color",'b');
xlim(axe_2,[-2,0]);
ylim(axe_2,[-1.5,-0.5]);
box(axe_2,"on");
grid(axe_2,"on");
