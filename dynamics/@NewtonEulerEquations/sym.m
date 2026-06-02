function eqns = sym(obj,options)
    arguments
        obj (1,1) NewtonEulerEquations;
        options.SmallAng (:,1) = sym.empty(0,1);
    end

    G = obj.SpatialInertia;
    V = obj.Twist;
    Vd = obj.TwistRate;
    wm = vec2skew(V(1:3));
    adV = blkdiag(wm,wm);

    f = @(x)x;
    if ~isempty(options.SmallAng)
        f = @(x)smallang(x,options.SmallAng);
    end

    R = obj.Pose(1:3,1:3);
    p = obj.Pose(1:3,4);

    Ad = [
        R.',zeros(3);
        -f(R.'*vec2skew(p)),R.'
        ];

    eqns = f(Ad.'*G*Vd) + f(Ad.'*adV*G*V) - f(Ad.'*obj.ActiveForces);
end