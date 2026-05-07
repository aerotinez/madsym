function out = smallang(obj,x)
    arguments
        obj (1,1) DynamicEquations
        x (:,1) sym
    end

    out = obj;

    out.SpatialInertia = smallang(obj.SpatialInertia,x);
    out.Jacobian = smallang(obj.Jacobian,x);
    out.JacobianRate = smallang(obj.JacobianRate,x);
    out.ActiveForces = smallang(obj.ActiveForces,x);
    out.ConstraintJacobian = smallang(obj.ConstraintJacobian,x);

    B = out.ConstraintJacobian;
    Bind = simplify(expand(B(:,1:numel(out.States.independent))));
    Bdep = simplify(expand(B(:,numel(out.States.independent) + 1:end)));
    A = syminv(Bdep);
    Jc = smallang(-A*Bind,x);

    M = smallang(out.Jacobian.'*out.SpatialInertia*out.Jacobian,x);

    if ~isequal(Jc, eye(numel(obj.States),'sym'))
        n = numel(obj.States);
        m = size(Jc,1);
        k = n - m;

        M = M(1:k,:) + Jc.'*M(k+1:end,:);
    end

    out.MassMatrix = smallang(M,x);

    if obj.IsTrimmed
        u = [obj.States.TrimState].';
    else
        u = state(obj.States);
    end

    J  = out.Jacobian;
    dJ = out.JacobianRate;
    G  = out.SpatialInertia;
    W  = out.ActiveForces;

    V = smallang(J*u,x);

    nb = size(G,1)/6;
    adw = sym.zeros(6*nb);

    for i = 1:nb
        idx = (6*(i - 1) + 1):(6*i);
        w = V(idx(1:3));
        adw(idx,idx) = blkdiag(vec2skew(w),vec2skew(w));
    end

    f0 = smallang(J.'*W,x);
    f1 = smallang(-J.'*G*dJ*u,x);
    f2 = smallang(-J.'*adw*G*J*u,x);
    f = f0 + f1 + f2;

    if ~isequal(Jc, eye(numel(obj.States),'sym'))
        n = numel(obj.States);
        m = size(Jc,1);
        k = n - m;

        f = f(1:k) + Jc.'*f(k+1:end);
    end

    out.ForcingVector = smallang(f,x);
end