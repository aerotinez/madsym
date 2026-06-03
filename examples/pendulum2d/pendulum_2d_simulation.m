close("all"); clear; clc;

l = 1;
g = 9.81;

filename = "pendulum_vxy.h5";

Ntraj = 100;
ns = 1000;
tspan = linspace(0,10,ns);

for k = 1:Ntraj
    x0 = [
        deg2rad(-40 + 80*rand)
        deg2rad(-40 + 80*rand)
        -1 + 2*rand
        -1 + 2*rand
    ];

    f = @(t,y) pendulum2d(y,[l,g]');
    [t,y] = ode45(f,tspan,x0);

    yd = zeros(size(y));
    for i = 1:length(t)
        yd(i,:) = f(t(i), y(i,:).').';
    end

    group = sprintf("/traj_%04d", k);

    h5create(filename, group + "/t", size(t));
    h5write(filename, group + "/t", t);

    h5create(filename, group + "/x", size(y));
    h5write(filename, group + "/x", y);

    h5create(filename, group + "/xdot", size(yd));
    h5write(filename, group + "/xdot", yd);
end