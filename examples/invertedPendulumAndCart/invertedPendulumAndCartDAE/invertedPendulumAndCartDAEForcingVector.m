function f = invertedPendulumAndCartDAEForcingVector(x,u,p)
    fk = invertedPendulumAndCartKinematicForcingVector(x(1:2),x(3:end),p);
    fd1 = invertedPendulumAndCartBody1DynamicsForcingVector(x,u,p);
    fd2 = invertedPendulumAndCartBody2DynamicsForcingVector(x,u,p);
    fd = fd1 + fd2;
    f = [
        fk;
        fd;
        ];
end
