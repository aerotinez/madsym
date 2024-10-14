function eom = lagrangesMethod(q,u,kdes,bodies,F,cons,v,ades)
    arguments
        q (:,1) DynamicVariable;
        u (:,1) DynamicVariable;
        kdes (:,1) sym;
        bodies (:,1) Body;
        F (:,1) DynamicVariable = DynamicVariable.empty(0,1);
        cons (:,1) ConstraintEquations = ConstraintEquations.empty(0,1);
        v (:,1) DynamicVariable = DynamicVariable.empty(0,1);
        ades (:,1) sym = sym.empty(0,1);
    end
    ql = q.partition();
    ul = u.partition();
    eomk = KinematicEquations(ql,kdes,ul).simplify();
    eomc = ConstraintEquations.empty(0,1);
    Jc = sym.empty(0,1);
    if ~isempty(cons)
        eomc = partition(cons).reformulate(eomk).simplify(); 
        Jc = partitionJacobian(eomc);
    end
    eomd_list = arrayfun(@(b)bodyDynamics(b,eomk,permMat(u),F,Jc),bodies);
    eomv = MotionEquations.empty(0,1);
    if ~isempty(ades)
        eomv = auxiliaryEquations(v,ades,eomk,F);
    end
    eom = MechanicsEquations(eomk,eomd_list,eomc,eomv);
end

function eomd = bodyDynamics(body,eomk,P,inputs,Jc)
    arguments
        body (1,1) Body;
        eomk (1,1) KinematicEquations;
        P sym;
        inputs (:,1) DynamicVariable = DynamicVariable.empty(0,1);
        Jc sym = sym.empty(0,1);
    end
    V = body.Twist.vector;
    G = blkdiag(body.Inertia,body.Mass.*eye(3));
    L = (1/2).*V.'*G*V;

    dLdqd = P*jacobian(L,eomk.States.rate).';
    dLdq = P*jacobian(L,eomk.States.state).';
    t = sym('t');
    qdd = diff(eomk.States.rate,t);

    ovars = [
        qdd;
        eomk.States.rate;
        ];

    nvars = [
        diff(eomk.ForcingVector,t);
        eomk.ForcingVector;
        ];

    eqns = subs(diff(dLdqd,t) - dLdq,ovars,nvars);
    u = eomk.Inputs;
    [M,f0] = massMatrixForm(eqns,u.state);

    f1 = zeros(size(f0),"sym");

    J = body.Twist.jacobian(eomk.States);
    T = Pose(body.ReferenceFrame,body.MassCenter);
    W = body.ActiveForces.vector(T);
    f2 = P*J.'*subs(W,eomk.States.rate,eomk.ForcingVector);

    eomd = DynamicEquations(eomk.Inputs,M,f0,f1,f2,inputs);

    if ~isempty(Jc)
        eomd = constrain(eomd,Jc); 
    end
end 