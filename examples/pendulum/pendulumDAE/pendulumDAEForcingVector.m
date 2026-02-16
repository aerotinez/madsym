function f = pendulumDAEForcingVector(x,u,p)
    fk = pendulumKinematicForcingVector(x(1:1),x(2:end),p);
    fd1 = pendulumBody1DynamicsForcingVector(x,u,p);
    fd = fd1;
    f = [
        fk;
        fd;
        ];
end
