function eoml = linearize(obj,x,u)
    arguments
        obj (1,1) DynamicEquations
        x (:,1) DynamicVariable = obj.States
        u (:,1) DynamicVariable = obj.Inputs
    end
    
    vars = [
        x;
        u
        ];

    q = obj.States.state;

    G  = obj.SpatialInertia;
    J  = obj.Jacobian;
    dJ = obj.JacobianRate;
    W  = obj.ActiveForces;

    V = J*q;

    nb = size(G,1)/6;
    adw = sym.zeros(6*nb,6*nb);

    for i = 1:nb
        idx = (6*(i-1)+1):(6*i);
        w = V(idx(1:3));
        adw(idx,idx) = blkdiag(vec2skew(w),vec2skew(w));
    end

    M  = J.'*G*J;
    f0 = -J.'*G*dJ*q;
    f1 = -J.'*adw*G*J*q;
    f2 =  J.'*W;

    Jc = obj.constrainingJacobian();

    if ~isequal(Jc,eye(numel(obj.States),'sym'))
        n = numel(obj.States);
        m = size(Jc,1);
        k = n - m;

        M  = M(1:k,:)  + Jc.'*M(k+1:end,:);
        f0 = f0(1:k)   + Jc.'*f0(k+1:end);
        f1 = f1(1:k)   + Jc.'*f1(k+1:end);
        f2 = f2(1:k)   + Jc.'*f2(k+1:end);
    end

    fM = M*obj.States.rate;
    Mlin = subsTrim(jacobian(fM,x.rate),vars);
    
    f = -(f0 + f1 + f2);

    JfM = subsTrim(jacobian(fM,x.state),vars);
    Jf0 = subsTrim(jacobian(-f0,x.state),vars);
    Jf1 = subsTrim(jacobian(-f1,x.state),vars);
    Jf2 = subsTrim(jacobian(-f2,x.state),vars);

    Glin = -subsTrim(jacobian(f,u.state),vars);
    Hlin = -(JfM + Jf0 + Jf1 + Jf2);

    eoml = LinearizedMotionEquations(x,Mlin,Hlin,Glin,u);
end