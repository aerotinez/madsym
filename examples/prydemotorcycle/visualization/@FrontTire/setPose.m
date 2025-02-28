function setPose(obj,q)
    arguments
        obj (1,1) FrontTire;
        q (1,4) double;
    end
    p = obj.GeometricParameters;

    yaw = q(1);
    camber = q(2);
    pitch = q(3);
    steer = q(4);

    I = eye(3);
    n0 = zeros(1,3);

    f = @(R,d)rigidtform3d(R,d).A;
    
    A = f(rotz(yaw),n0);

    d = [0,0,p.RearCrownRadius];

    A = A*f(I,d);
    A = A*f(rotx(camber),n0);

    d = [0,0,p.RearRadius - p.RearCrownRadius];

    A = A*f(I,d);
    A = A*f(roty(pitch),n0);

    d = [p.Wheelbase,0,p.FrontRadius];
    A = A*f(I,d);

    A = A*f(roty(p.Caster).',n0);

    A = A*f(I,[-p.Rake,0,0]);

    A = A*f(rotz(steer),n0);
    
    A = A*f(I,[p.Rake,0,0]);

    setPose@MotoBody(obj,A);
end