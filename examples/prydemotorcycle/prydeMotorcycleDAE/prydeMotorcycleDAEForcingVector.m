function f = prydeMotorcycleDAEForcingVector(x,u,p)
    fk = prydeMotorcycleKinematicForcingVector(x(1:12),x(13:end),p);
    fd1 = prydeMotorcycleBody1DynamicsForcingVector(x,u,p);
    fd2 = prydeMotorcycleBody2DynamicsForcingVector(x,u,p);
    fd3 = prydeMotorcycleBody3DynamicsForcingVector(x,u,p);
    fd4 = prydeMotorcycleBody4DynamicsForcingVector(x,u,p);
    fd = fd1 + fd2 + fd3 + fd4;
    fc = prydeMotorcycleConstraintForcingVector(x(1:12),x(13:end),p);
    f = [
        fk;
        fd;
        fc
        ];
end
