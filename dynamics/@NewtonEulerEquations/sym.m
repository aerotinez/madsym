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

    eqns = f(G*Vd) + f(adV*G*V) - obj.ActiveForces;
end