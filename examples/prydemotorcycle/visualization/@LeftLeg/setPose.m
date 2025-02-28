function setPose(obj,q)
    arguments
        obj (1,1) LeftLeg;
        q (1,4) double;
    end
    p = obj.GeometricParameters;

    yaw = q(1);
    camber = q(2);
    pitch = q(3);

    I = eye(3);
    n0 = zeros(1,3);

    f = @(R,d)rigidtform3d(R,d).A;
    
    A = f(rotz(yaw),n0);

    d = [0,0,p.RearCrownRadius];

    A = A*f(I,d);
    A = A*f(rotx(camber),n0);

    A = A*f(roty(pitch),n0);

    d = [
        p.Wheelbase - p.StepOffset;
        p.RiderWidth/2;
        p.StepHeight + p.ShinHeight - p.RearCrownRadius
        ].';

    A = A*f(I,d);

    A = A*f(I,roty(p.RiderLean)*[-p.RiderDepth/2,0,0].');

    setPose@MotoBody(obj,0.*A + eye(4));
end