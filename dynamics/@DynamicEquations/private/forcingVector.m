function f = forcingVector(obj)
    if obj.IsTrimmed
        u = [obj.States.TrimState].';
    else
        u = state(obj.States);
    end

    J  = obj.Jacobian;
    dJ = obj.JacobianRate;
    G  = obj.SpatialInertia;
    W  = obj.ActiveForces;

    V = J*u;

    nb = size(G,1)/6;
    adw = sym.zeros(6*nb);

    for i = 1:nb
        idx = (6*(i - 1) + 1):(6*i);
        w = V(idx(1:3));
        adw(idx,idx) = blkdiag(vec2skew(w),vec2skew(w));
    end

    f = J.'*(W - (G*dJ + adw*G*J)*u);

    Jc = obj.constrainingJacobian();

    if ~isequal(Jc, eye(numel(obj.States),'sym'))
        n = numel(obj.States);
        m = size(Jc,1);
        k = n - m;

        f = f(1:k) + Jc.'*f(k+1:end);
    end
end