function eom = reduce(obj,x,u)
    arguments
        obj (1,1) LinearizedMotionEquations;
        x (:,1) DynamicVariable = obj.States;
        u (:,1) DynamicVariable = obj.Inputs;
    end
    P = jacobian(x.state,obj.States.state);
    eqns = P*sym(obj);
    [M,f] = massMatrixForm(eqns,x.state);
    [H,g] = equationsToMatrix(f,x.state);
    G = jacobian(-g,u.state);
    eom = LinearizedMotionEquations(x,M,H,G,u);
end
