close("all"); clear; clc;

l = 1;
g = 9.81;

filename = "pendulum_2d_cranmer.h5";

if isfile(filename)
    delete(filename);
end

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

    [t,x] = ode45(f,tspan,x0);

    xdot = zeros(size(x));

    for i = 1:length(t)
        xdot(i,:) = f(t(i),x(i,:).').';
    end

    group = sprintf("/traj_%04d",k);

    h5create(filename,group+"/t",size(t));
    h5write(filename,group+"/t",t);

    names = ["q1","q2","w1","w2"];
    dnames = ["q1dot","q2dot","w1dot","w2dot"];

    for j = 1:4

        h5create(filename,group+"/"+names(j),size(x(:,j)));
        h5write(filename,group+"/"+names(j),x(:,j));

        h5create(filename,group+"/"+dnames(j),size(xdot(:,j)));
        h5write(filename,group+"/"+dnames(j),xdot(:,j));

    end

    h5writeatt(filename,group,"l",l);
    h5writeatt(filename,group,"g",g);
    h5writeatt(filename,group,"x0",x0);

end